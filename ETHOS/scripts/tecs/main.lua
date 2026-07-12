-- TECS tuning advisor for FrSky Ethos (X20S / X18 / X18S ...)  v0.3.1
-- Ethos port of the OpenTX/EdgeTX "Arduplane TECS tuning helper" widget.
--
-- This program is free software; you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation; either version 3 of the License, or
-- (at your option) any later version.
--
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY, without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
-- GNU General Public License for more details.
--
-- You should have received a copy of the GNU General Public License
-- along with this program; if not, see <http://www.gnu.org/licenses>.

-- ================================================================
-- CONFIG / CONSTANTS
-- ================================================================

-- absolute path to this script's audio folder (Ethos wants full paths)
local AUDIO = "/scripts/tecs/audio/en/"

-- CRSF ArduPilot passthrough framing (same as OpenTX crossfireTelemetryPop)
local CRSF_FRAME_CUSTOM_TELEM         = 0x80
local CRSF_FRAME_CUSTOM_TELEM_LEGACY  = 0x7F
local CRSF_PASSTHROUGH                = 0xF0
local CRSF_PASSTHROUGH_ARRAY          = 0xF2

local STEP_COUNT = 7      -- number of tuning steps
local TRIGGER_HOLDOFF = 1.5 -- seconds, debounce between switch activations

-- ================================================================
-- CRSF sensor (Ethos exposes crsf.getSensor() on newer builds)
-- ================================================================
local crsfSensor = {}
if crsf == nil then
  -- CRSF module not available on this build; provide a stub
  function crsfSensor:popFrame() return nil, nil end
elseif crsf.getSensor == nil then
  function crsfSensor:popFrame() return crsf.popFrame() end
else
  crsfSensor = crsf.getSensor()
end

-- ================================================================
-- TELEMETRY
-- ================================================================
local telemetry = {
  roll     = 0,   -- deg
  pitch    = 0,   -- deg
  vSpeed   = 0,   -- dm/s (climb/sink rate)
  hSpeed   = 0,   -- dm/s (groundspeed)
  airspeed = 0,   -- dm/s
  throttle = 0,   -- percent (from VFR frame)
}

local telemetryOk = false
local lastTelemetry = 0    -- os.clock() timestamp of last decoded frame

-- native 5.4 bit extraction (OpenTX used bit32.extract)
local function bitExtract(value, start, len)
  return (value >> start) & ((1 << len) - 1)
end

local function processTelemetry(appId, value)
  if appId == 0x5006 then          -- ROLLPITCH
    telemetry.roll  = (math.min(bitExtract(value, 0, 11), 1800) - 900) * 0.2
    telemetry.pitch = (math.min(bitExtract(value, 11, 10), 900) - 450) * 0.2
  elseif appId == 0x5005 then      -- VELANDYAW
    telemetry.vSpeed = bitExtract(value, 1, 7) * (10 ^ bitExtract(value, 0, 1))
                       * (bitExtract(value, 8, 1) == 1 and -1 or 1)
    if bitExtract(value, 28, 1) == 1 then
      telemetry.airspeed = bitExtract(value, 10, 7) * (10 ^ bitExtract(value, 9, 1))
    else
      telemetry.hSpeed = bitExtract(value, 10, 7) * (10 ^ bitExtract(value, 9, 1))
    end
  elseif appId == 0x50F2 then      -- VFR
    telemetry.airspeed = bitExtract(value, 1, 7) * (10 ^ bitExtract(value, 0, 1))
    telemetry.throttle = bitExtract(value, 8, 7)
  end
end

-- pops and decodes waiting CRSF frames (ArduPilot passthrough)
local function crossfirePop()
  local command, data = crsfSensor:popFrame()
  if command == nil or data == nil then
    return false
  end
  if command == CRSF_FRAME_CUSTOM_TELEM or command == CRSF_FRAME_CUSTOM_TELEM_LEGACY then
    if #data >= 7 and data[1] == CRSF_PASSTHROUGH then
      local appId = (data[3] << 8) + data[2]
      local value = (data[7] << 24) + (data[6] << 16) + (data[5] << 8) + data[4]
      processTelemetry(appId, value)
      return true
    elseif #data >= 8 and data[1] == CRSF_PASSTHROUGH_ARRAY then
      for i = 0, math.min(data[2] - 1, 9) do
        local appId = (data[4 + (6 * i)] << 8) + data[3 + (6 * i)]
        local value = (data[8 + (6 * i)] << 24) + (data[7 + (6 * i)] << 16)
                      + (data[6 + (6 * i)] << 8) + data[5 + (6 * i)]
        processTelemetry(appId, value)
      end
      return true
    end
  end
  return false
end

-- ================================================================
-- FrSky SPort passthrough (R9 / X-S-series / F.Port)  -- UNTESTED on hardware
-- ================================================================
-- Ethos has no sportTelemetryPop(); instead ArduPilot's passthrough app-ids are
-- discovered as DIY telemetry sensors. We read each sensor's raw 32-bit value and
-- feed it to the SAME processTelemetry() the CRSF path uses. Requires:
--   * aircraft serial SERIALx_PROTOCOL = 10 (Frsky SPort passthrough), BAUD 57
--   * the 0x50xx sensors discovered once on the radio's telemetry page
-- NOTE: this relies on Ethos returning the raw uint32 payload for these DIY
-- sensors (masked to 32 bits below). Confirm on a bench with an R9 + FC before
-- trusting the numbers.
local SPORT_APPIDS = { 0x5006, 0x5005, 0x50F2 }  -- same ids processTelemetry() decodes
local sportSources = nil   -- lazily-resolved { appId -> source }, nil until first poll
local sportLast    = {}    -- { appId -> last raw value } for change detection

local function sportPoll()
  if sportSources == nil then sportSources = {} end
  local got = false
  for _, appId in ipairs(SPORT_APPIDS) do
    local src = sportSources[appId]
    if src == nil then
      -- not resolved yet (or sensor not discovered): try again this poll
      src = system.getSource({ appId = appId })
      sportSources[appId] = src
    end
    if src ~= nil then
      local value = src:value()
      if value ~= nil then
        local raw = math.floor(value) & 0xFFFFFFFF   -- integer, masked to 32 bits
        processTelemetry(appId, raw)                 -- refresh even if unchanged
        if raw ~= sportLast[appId] then              -- but only "live" counts as new data
          sportLast[appId] = raw
          got = true
        end
      end
    end
  end
  return got
end

-- ================================================================
-- UNIT CONVERSIONS (raw telemetry -> Arduplane parameter units)
-- ================================================================
local function dmsToMs(dm)   return dm * 0.1  end          -- dm/s -> m/s
local function dmsToKph(dm)  return dm * 0.36 end          -- dm/s -> km/h
local function clampMs(dm)   return math.min(math.abs(0.1 * dm), 10) end -- dm/s -> +m/s (<=10)

-- ================================================================
-- TECS PARAMETERS
-- value = captured raw value, exporter = raw -> Arduplane unit
-- ================================================================
-- NOTE: names/units target ArduPlane 4.5+ (verified against 4.6.3). The airspeed
-- params were renamed from ARSPD_FBW_MIN/MAX and TRIM_ARSPD_CM, and AIRSPEED_CRUISE
-- is now m/s (was cm/s), so its exporter converts dm/s -> m/s, not dm/s -> cm/s.
local TECS = {
  TRIM_THROTTLE   = { value = 0,  exporter = function(v) return v end },                 -- percent
  AIRSPEED_CRUISE = { value = 0,  exporter = function(v) return dmsToMs(v) end },         -- dm/s -> m/s
  THR_MAX         = { value = 0,  exporter = function(v) return v end },                  -- percent
  AIRSPEED_MAX    = { value = 0,  exporter = function(v) return dmsToMs(v * 0.95) end },  -- dm/s -> m/s * 0.95
  TECS_PITCH_MAX  = { value = -4, exporter = function(v) return math.abs(v + 4) end },    -- +deg  (-4 margin)
  TECS_CLMB_MAX   = { value = 0,  exporter = function(v) return clampMs(v) end },         -- +m/s
  FBWB_CLIMB_RATE = { value = 0,  exporter = function(v) return clampMs(v) end },         -- m/s
  AIRSPEED_MIN    = { value = 0,  exporter = function(v) return dmsToMs(v) end },         -- m/s
  STAB_PITCH_DOWN = { value = 0,  exporter = function(v) return math.abs(v) end },        -- deg
  TECS_SINK_MIN   = { value = 0,  exporter = function(v) return clampMs(v) end },         -- m/s
  TECS_PITCH_MIN  = { value = 4,  exporter = function(v) return v - 4 end },              -- -deg  (+4 margin)
  TECS_SINK_MAX   = { value = 0,  exporter = function(v) return clampMs(v) end },         -- +m/s
  KFF_THR2PTCH    = { value = 0,  exporter = function(v) return v end },                  -- deg
}

-- ordered list for display / logging
local TECS_ORDER = {
  "TRIM_THROTTLE", "AIRSPEED_CRUISE", "THR_MAX", "AIRSPEED_MAX",
  "TECS_PITCH_MAX", "TECS_CLMB_MAX", "FBWB_CLIMB_RATE", "AIRSPEED_MIN",
  "STAB_PITCH_DOWN", "TECS_SINK_MIN", "TECS_PITCH_MIN", "TECS_SINK_MAX",
  "KFF_THR2PTCH",
}

local function exportTECS(name)
  local p = TECS[name]
  if p == nil then return 0 end
  return p.exporter(p.value)
end

-- rounds an exported value for display (integers where sensible, else 1 decimal)
local function fmt(v)
  if v == math.floor(v) then
    return string.format("%d", v)
  end
  return string.format("%.1f", v)
end

-- read throttle as percent: from configured stick source, else from VFR telemetry
local function getThrottlePct(widget)
  if widget.throttleSource ~= nil then
    local v = widget.throttleSource:value()   -- stick -1024..1024
    return math.floor((v + 1024) / 20.48)
  end
  return telemetry.throttle                    -- VFR reports 0..100 directly
end

-- ================================================================
-- TUNING STEPS  (audio instructions + value capture)
-- ================================================================
local function playFile(f) system.playFile(AUDIO .. f) end
local function capArspd() -- airspeed if available, else groundspeed (dm/s)
  return (telemetry.airspeed ~= 0 and telemetry.airspeed) or telemetry.hSpeed
end

local stepDef = {
  [1] = {
    text = "Continue in Fly by Wire A and fly level at desired cruise speed.",
    audio = function() playFile("tecs10.wav") end,
    fn = function(widget)
      TECS.TRIM_THROTTLE.value = getThrottlePct(widget)
      TECS.AIRSPEED_CRUISE.value = capArspd()
    end,
  },
  [2] = {
    text = "Now accelerate to your desired maximum cruise speed.",
    audio = function()
      playFile("tecs11.wav")
      system.playNumber(dmsToKph(TECS.AIRSPEED_CRUISE.value), UNIT_KPH, 0)
      playFile("tecs20.wav")
    end,
    fn = function(widget)
      TECS.THR_MAX.value       = getThrottlePct(widget)
      TECS.AIRSPEED_MAX.value = capArspd()
    end,
  },
  [3] = {
    text = "Keep the throttle and start climbing until airspeed reaches cruise speed.",
    audio = function()
      playFile("tecs21.wav")
      system.playNumber(dmsToKph(TECS.AIRSPEED_MAX.value), UNIT_KPH, 0)
      playFile("tecs30.wav")
      system.playNumber(TECS.THR_MAX.value, UNIT_PERCENT, 0)
      playFile("tecs31.wav")
      system.playNumber(dmsToKph(TECS.AIRSPEED_CRUISE.value), UNIT_KPH, 0)
    end,
    fn = function(widget)
      TECS.TECS_PITCH_MAX.value  = telemetry.pitch
      TECS.TECS_CLMB_MAX.value   = telemetry.vSpeed
      TECS.FBWB_CLIMB_RATE.value = telemetry.vSpeed
    end,
  },
  [4] = {
    text = "Slow down to the minimum safe speed without stalling.",
    audio = function()
      playFile("tecs32.wav")
      system.playNumber(clampMs(TECS.TECS_CLMB_MAX.value), UNIT_METER_PER_SECOND, 0)
      playFile("tecs40.wav")
    end,
    fn = function(widget)
      TECS.AIRSPEED_MIN.value = capArspd()
    end,
  },
  [5] = {
    text = "Gain altitude, then cut throttle and pitch down until airspeed reaches min speed.",
    audio = function()
      playFile("tecs41.wav")
      system.playNumber(dmsToKph(TECS.AIRSPEED_MIN.value), UNIT_KPH, 0)
      playFile("tecs50.wav")
      system.playNumber(dmsToKph(TECS.AIRSPEED_MIN.value), UNIT_KPH, 0)
    end,
    fn = function(widget)
      TECS.STAB_PITCH_DOWN.value = telemetry.pitch
      TECS.TECS_SINK_MIN.value   = telemetry.vSpeed
    end,
  },
  [6] = {
    text = "Continue with zero throttle and pitch down until airspeed reaches max speed.",
    audio = function()
      playFile("tecs51.wav")
      system.playNumber(clampMs(TECS.TECS_SINK_MIN.value), UNIT_METER_PER_SECOND, 0)
      playFile("tecs60.wav")
      system.playNumber(dmsToKph(TECS.AIRSPEED_MAX.value), UNIT_KPH, 0)
    end,
    fn = function(widget)
      TECS.TECS_PITCH_MIN.value = telemetry.pitch
      TECS.TECS_SINK_MAX.value  = telemetry.vSpeed
    end,
  },
  [7] = {
    text = "Fly full speed and try to hold altitude.",
    audio = function()
      playFile("tecs61.wav")
      system.playNumber(clampMs(TECS.TECS_SINK_MAX.value), UNIT_METER_PER_SECOND, 0)
      playFile("tecs70.wav")
    end,
    fn = function(widget)
      TECS.KFF_THR2PTCH.value = telemetry.pitch
        - math.sqrt((TECS.TRIM_THROTTLE.value - getThrottlePct(widget))
                    / (TECS.TRIM_THROTTLE.value - 100))
    end,
  },
}

-- ================================================================
-- LOGGING  ->  /scripts/tecs/tecs_<timestamp>.txt
-- ================================================================
local function timestamp()
  -- os.date may not be exposed on every Ethos build; fall back to a clock counter
  local ok, d = pcall(os.date, "*t")
  if ok and type(d) == "table" then
    return string.format("%04d%02d%02d_%02d%02d", d.year, d.month, d.day, d.hour, d.min)
  end
  return string.format("%d", math.floor(os.clock()))
end

local function logTECS()
  local f = io.open("/scripts/tecs/tecs_" .. timestamp() .. ".txt", "a")
  if f == nil then return end
  for _, name in ipairs(TECS_ORDER) do
    io.write(f, string.format("%s=%s\r\n", name, fmt(exportTECS(name))))
  end
  -- raw captured values for debugging
  for _, name in ipairs(TECS_ORDER) do
    io.write(f, string.format("debug_%s=%s\r\n", name, tostring(TECS[name].value)))
  end
  io.close(f)
end

-- ================================================================
-- STEP MACHINE  (mirrors the OpenTX manual_trigger logic)
-- returns a short status string for display
-- ================================================================
local function advance(widget)
  if widget.step == 1 then
    stepDef[1].audio()
    widget.step = 2
  else
    local prev = widget.step - 1
    stepDef[prev].fn(widget)                 -- capture values from previous step
    if widget.step > STEP_COUNT then         -- finished: reset + summary
      widget.step = 1
      logTECS()
      playFile("tecsf.wav")
    else
      stepDef[widget.step].audio()
      widget.step = widget.step + 1
    end
  end
end

-- rising-edge + hold-off detection on the trigger switch
local function checkTrigger(widget)
  if widget.switchSource == nil then return end
  local now = os.clock()
  local active = widget.switchSource:value() > 0
  if active and not widget.switchWasActive and (now - widget.lastTrigger) > TRIGGER_HOLDOFF then
    widget.lastTrigger = now
    advance(widget)
  end
  widget.switchWasActive = active
end

-- ================================================================
-- WIDGET LIFECYCLE
-- ================================================================
local function create()
  return {
    -- config (persisted)
    switchSource   = nil,
    throttleSource = nil,
    backColor      = lcd.RGB(0, 0, 0),
    foreColor      = lcd.RGB(255, 255, 255),
    useSport       = false,   -- false = CRSF frames (Crossfire/ELRS), true = FrSky SPort sensors (R9)
    -- runtime state
    step           = 1,
    lastTrigger    = 0,
    switchWasActive = false,
  }
end

local function wakeup(widget)
  local got = false
  if widget.useSport then
    -- FrSky SPort: poll discovered passthrough sensors (R9 / X-S-series / F.Port)
    got = sportPoll()
  else
    -- drain any waiting CRSF frames (this replaces the OpenTX background())
    for _ = 1, 20 do
      if not crossfirePop() then break end
      got = true
    end
  end
  if got then
    lastTelemetry = os.clock()
    telemetryOk = true
  elseif (os.clock() - lastTelemetry) > 2 then
    telemetryOk = false
  end

  checkTrigger(widget)
  lcd.invalidate()
end

local function paint(widget)
  local w, h = lcd.getWindowSize()

  -- background
  lcd.color(widget.backColor)
  lcd.drawFilledRectangle(0, 0, w, h)
  lcd.color(widget.foreColor)

  -- adaptive geometry: telemetry on the left, TECS params on the right
  local row    = math.max(14, math.floor((h - 24) / 14))
  local colX   = math.floor(w * 0.40)   -- start of param column
  local valX   = w - 8                  -- right-aligned values
  local telX   = 6
  local telValX = colX - 12

  lcd.font(FONT_S)

  local y = 2
  lcd.drawText(telX, y, "= TECS TUNING =")

  -- --- telemetry block ---
  local function kv(x, xv, yy, label, value)
    lcd.drawText(x, yy, label)
    lcd.drawText(xv, yy, value, RIGHT)
  end

  y = y + row + 4
  lcd.drawText(telX, y, "telemetry:")
  y = y + row
  kv(telX, telValX, y, "pitch",       fmt(telemetry.pitch)); y = y + row
  kv(telX, telValX, y, "roll",        fmt(telemetry.roll)); y = y + row
  kv(telX, telValX, y, "gspeed kph",  fmt(dmsToKph(telemetry.hSpeed))); y = y + row
  kv(telX, telValX, y, "aspeed kph",  fmt(dmsToKph(telemetry.airspeed))); y = y + row
  kv(telX, telValX, y, "climb m/s",   fmt(dmsToMs(telemetry.vSpeed))); y = y + row + 4

  lcd.drawText(telX, y, "info:"); y = y + row
  kv(telX, telValX, y, "telemetry", telemetryOk and "OK" or "--"); y = y + row
  kv(telX, telValX, y, "step",      string.format("%d/%d", widget.step, STEP_COUNT + 1)); y = y + row
  if widget.switchSource == nil then
    lcd.drawText(telX, y, "set trigger switch!")
  end

  -- --- TECS params block (right) ---
  local py = 2
  for _, name in ipairs(TECS_ORDER) do
    lcd.drawText(colX, py, name)
    lcd.drawText(valX, py, fmt(exportTECS(name)), RIGHT)
    py = py + row
  end

  -- --- current instruction (bottom, centered) ---
  lcd.font(FONT_XS)
  local instruction
  if widget.step == 1 then
    instruction = "engage the trigger switch to start"
  else
    instruction = stepDef[widget.step - 1].text
  end
  lcd.drawText(math.floor(w / 2), h - row, instruction, CENTERED)
end

-- ================================================================
-- CONFIGURATION FORM
-- ================================================================
local function configure(widget)
  -- a switch is selectable as a source; addSourceField is the portable API
  local line = form.addLine("Trigger switch")
  form.addSourceField(line, nil,
    function() return widget.switchSource end,
    function(v) widget.switchSource = v end)

  line = form.addLine("Throttle source (optional)")
  form.addSourceField(line, nil,
    function() return widget.throttleSource end,
    function(v) widget.throttleSource = v end)

  -- OFF = CRSF passthrough (Crossfire/ELRS); ON = FrSky SPort passthrough (R9 etc.)
  line = form.addLine("FrSky SPort link (R9)")
  form.addBooleanField(line, nil,
    function() return widget.useSport end,
    function(v) widget.useSport = v end)
end

-- ================================================================
-- PERSISTENCE
-- ================================================================
local function read(widget)
  widget.switchSource   = storage.read("switchSource")
  widget.throttleSource = storage.read("throttleSource")
  widget.useSport       = storage.read("useSport") or false
end

local function write(widget)
  storage.write("switchSource", widget.switchSource)
  storage.write("throttleSource", widget.throttleSource)
  storage.write("useSport", widget.useSport)
end

-- ================================================================
-- REGISTRATION
-- ================================================================
local function init()
  system.registerWidget({
    key       = "tecs",          -- max 7 chars
    name      = "TECS Tuning",
    create    = create,
    paint     = paint,
    wakeup    = wakeup,
    configure = configure,
    read      = read,
    write     = write,
  })
end

return { init = init }
