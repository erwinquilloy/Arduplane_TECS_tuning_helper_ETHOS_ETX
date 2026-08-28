-- TECS tuning advisor for FrSky Ethos (X20S / X18 / X18S ...)  v0.5.0
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

local crsfPops   = 0      -- CRSF frames popped since start (debug)
local crsfFrames = 0      -- CRSF frames carrying passthrough app-ids (debug)

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
  throttle = 0,   -- percent, signed (from the 0x5001 AP_STATUS frame)
  mode     = -1,  -- ArduPlane mode number, -1 until a frame arrives
  armed    = 0,   -- 1 when the FC reports armed
}

local telemetryOk = false
local lastTelemetry = 0    -- os.clock() timestamp of last decoded frame

-- native 5.4 bit extraction (OpenTX used bit32.extract)
local function bitExtract(value, start, len)
  return (value >> start) & ((1 << len) - 1)
end

-- app-ids seen, recorded inside processTelemetry so the diagnostic page shows
-- whichever transport is live (the CRSF path never goes through sportPop)
local telemSeen      = {}   -- { "appId" -> last value }
local telemSeenOrder = {}   -- appIds in first-seen order

local function processTelemetry(appId, value)
  local key = string.format("%04X", appId)
  if telemSeen[key] == nil then telemSeenOrder[#telemSeenOrder + 1] = key end
  telemSeen[key] = value
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
  elseif appId == 0x5001 then      -- AP STATUS
    -- ArduPilot carries throttle here, NOT in 0x50F2: that app-id is a PX4
    -- packet and ArduPilot's PassthroughPacketType enum never emits it, so
    -- the branch below stays dead on a plane. 6-bit magnitude at 19 plus a
    -- sign bit at 25, sent as [-63,63] (AP_THROTTLE_OFFSET in
    -- AP_Frsky_SPort_Passthrough.cpp), rescaled here to [-100,100].
    -- ArduPilot encodes with prep_number(throttle*0.63, 2, 0), and that call
    -- passes a float into an int32_t parameter, so the wire value is
    -- floor(throttle * 0.63) -- truncated, not rounded like prep_number's other
    -- branches. Reconstructing at n/0.63 (what Yaapu does, as n*1.58) therefore
    -- sits on the bottom edge of each bucket and reads ~1% low across the whole
    -- range; +0.5 centres it. Against Mission Planner's unquantised value that
    -- is exact 64 times in 101 instead of 19, worst case 1% instead of 2%.
    -- n == 0 is special-cased: idle really is 0%, not the bucket midpoint.
    local thrRaw = bitExtract(value, 19, 6)
    local thrMag = 0
    if thrRaw > 0 then
      thrMag = math.min(100, math.floor(0.5 + (thrRaw + 0.5) / 0.63))
    end
    telemetry.throttle = thrMag * (bitExtract(value, 25, 1) == 1 and -1 or 1)
    telemetry.mode  = bitExtract(value, 0, 5)
    telemetry.armed = bitExtract(value, 8, 1)
  elseif appId == 0x50F2 then      -- VFR (PX4 only; kept for completeness)
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
  crsfPops = crsfPops + 1
  if command == CRSF_FRAME_CUSTOM_TELEM or command == CRSF_FRAME_CUSTOM_TELEM_LEGACY then
    if #data >= 7 and data[1] == CRSF_PASSTHROUGH then
      local appId = (data[3] << 8) + data[2]
      local value = (data[7] << 24) + (data[6] << 16) + (data[5] << 8) + data[4]
      crsfFrames = crsfFrames + 1
      processTelemetry(appId, value)
      return true
    elseif #data >= 8 and data[1] == CRSF_PASSTHROUGH_ARRAY then
      for i = 0, math.min(data[2] - 1, 9) do
        local appId = (data[4 + (6 * i)] << 8) + data[3 + (6 * i)]
        local value = (data[8 + (6 * i)] << 24) + (data[7 + (6 * i)] << 16)
                      + (data[6 + (6 * i)] << 8) + data[5 + (6 * i)]
        crsfFrames = crsfFrames + 1
        processTelemetry(appId, value)
      end
      return true
    end
  end
  return false
end

-- telemetry link selection (widget.linkMode)
local LINK_AUTO  = 0   -- drain both transports, whichever is live wins (default)
local LINK_CRSF  = 1   -- force CRSF frames only (Crossfire / ELRS)
local LINK_SPORT = 2   -- force FrSky SPort/F.Port frames only (R9 / X-S-series)

-- ================================================================
-- FrSky SPort / F.Port passthrough (R9 / X-S-series)
-- ================================================================
-- Ethos DOES expose raw SPort frames, via sport.getSensor() -> :popFrame().
-- (An earlier version of this file polled the discovered 0x50xx DIY sensors
-- with system.getSource() instead. That never worked: Ethos does not surface
-- the ArduPilot passthrough app-ids as telemetry sensors at all, so there was
-- nothing to read. This is the same API the Yaapu Ethos script uses.)
--
-- The sensor is a frame reader filtered to an appId range. ArduPilot passthrough
-- lives in 0x5000-0x50FF, plus GPS lat/lon on the standard 0x0800 -- the same
-- 0x800..0x51FF window Yaapu asks for. Note the range filter is unreliable on
-- some Ethos builds (FrSkyRC/ETHOS-Feedback-Community#1268) and hands back
-- frames outside the window, which is harmless: processTelemetry() ignores any
-- appId it does not know.
--
-- Requires nothing special on the aircraft beyond ArduPilot's normal FrSky
-- passthrough telemetry: SERIALx_PROTOCOL = 10 (SPort passthrough) on a wired
-- SmartPort, or SERIALx_PROTOCOL = 23 with half-duplex for F.Port, where
-- passthrough rides the RC link automatically.
local SPORT_PRIM_DATA = 0x10   -- SPort DATA_FRAME; polls/requests use other primIds
local SPORT_APPID_LO  = 0x800
local SPORT_APPID_HI  = 0x51FF

local sportReaders = {}     -- { name=, sensor=, pops=, frames= } per module
local sportTried   = false  -- so we only attempt to build it once
local sportFrames  = 0      -- frames popped since start (debug)
local sportErr     = nil    -- why the sensor could not be built (debug)
local sportPops    = 0      -- frames popped, before the primId filter (debug)
local sportNonData = 0      -- popped frames whose primId was not 0x10 (debug)
local sportPopErr  = nil    -- last error thrown by popFrame(), verbatim (debug)

-- builds the frame readers. sport.* is missing on very old Ethos builds, and an
-- error thrown inside wakeup() kills the widget silently, so this is guarded.
--
-- Ethos binds a reader to one RF module and offers no way to ask which one
-- carries the link, so both bindings are built and both are drained -- an
-- internal R9 and a module-bay ELRS then need no configuration. (Yaapu asks the
-- user instead: "internal module" passes no module key, "external module"
-- passes module=1.) The per-reader counters on the debug page say which is live.
local function sportInit()
  sportTried = true
  if sport == nil or sport.getSensor == nil then
    sportErr = "no sport API"
    return
  end
  local wanted = {
    { name = "dflt", arg = { appIdStart = SPORT_APPID_LO, appIdEnd = SPORT_APPID_HI } },
    { name = "mod1", arg = { module = 1,
                             appIdStart = SPORT_APPID_LO, appIdEnd = SPORT_APPID_HI } },
  }
  for _, w in ipairs(wanted) do
    local ok, sensor = pcall(sport.getSensor, w.arg)
    if ok and sensor ~= nil then
      sportReaders[#sportReaders + 1] =
        { name = w.name, sensor = sensor, pops = 0, frames = 0 }
    end
  end
  if #sportReaders == 0 then
    -- some builds reject the filter table; an unfiltered reader works too
    local ok, sensor = pcall(sport.getSensor)
    if ok and sensor ~= nil then
      sportReaders[1] = { name = "raw", sensor = sensor, pops = 0, frames = 0 }
    else
      sportErr = "getSensor failed"
    end
  end
end

-- pops one waiting frame off `r` and decodes it. Returns true if any frame was
-- consumed, so the caller keeps draining while the queue has more.
local function sportPopOne(r)
  -- the thrown message is kept, not discarded: on Ethos 1.6.x popFrame can be
  -- nil, which is indistinguishable from an idle link once the error is gone
  local ok, frame = pcall(function() return r.sensor:popFrame() end)
  if not ok then
    sportPopErr = tostring(frame)
    return false
  end
  if frame == nil then return false end
  r.pops = r.pops + 1
  sportPops = sportPops + 1

  local primId = frame:primId()
  if primId ~= SPORT_PRIM_DATA then
    sportNonData = sportNonData + 1
    return true   -- a real frame, just not a data frame: keep draining
  end

  local appId = frame:appId()
  local value = frame:value() & 0xFFFFFFFF

  r.frames = r.frames + 1
  sportFrames = sportFrames + 1

  processTelemetry(appId, value)
  return true
end

-- drains one frame from every reader. Returns true if any of them yielded.
local function sportPop()
  if #sportReaders == 0 then
    if sportTried then return false end
    sportInit()
    if #sportReaders == 0 then return false end
  end

  local any = false
  for _, r in ipairs(sportReaders) do
    if sportPopOne(r) then any = true end
  end
  return any
end

-- ArduPlane mode numbers (ArduPlane/mode.h). Only 5 bits are sent, so 0-31.
-- Worth showing by name: throttle telemetry reads 0 while armed on the ground in
-- any auto-throttle mode, so "LOITER" instead of "FBWA" explains a dead reading.
local MODE_NAMES = {
  [0]="MANUAL", [1]="CIRCLE", [2]="STAB", [3]="TRAIN", [4]="ACRO",
  [5]="FBWA", [6]="FBWB", [7]="CRUISE", [8]="AUTOTUNE", [10]="AUTO",
  [11]="RTL", [12]="LOITER", [13]="TAKEOFF", [14]="AVOID", [15]="GUIDED",
  [16]="INIT", [17]="QSTAB", [18]="QHOVER", [19]="QLOITER", [20]="QLAND",
  [21]="QRTL", [22]="QAUTOTUNE", [23]="QACRO", [24]="THERMAL",
  [25]="LOITER_QLAND", [26]="COURSE_HOLD", [27]="AUTO_TRIM",
}

local function modeName(m)
  if m < 0 then return "--" end
  return MODE_NAMES[m] or string.format("mode %d", m)
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

-- airspeed params are exported in m/s; we also show the km/h equivalent in
-- parentheses so the value can be cross-checked against a km/h OSD.
local SPEED_PARAMS = {
  AIRSPEED_CRUISE = true,
  AIRSPEED_MAX    = true,
  AIRSPEED_MIN    = true,
}

-- formats an exported param for display; airspeeds get a "(NN kph)" suffix
local function fmtParam(name)
  local v = exportTECS(name)
  if SPEED_PARAMS[name] then
    return string.format("%s (%skph)", fmt(v), fmt(v * 3.6))
  end
  return fmt(v)
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
      TECS.TRIM_THROTTLE.value = telemetry.throttle
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
      TECS.THR_MAX.value       = telemetry.throttle
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
        - math.sqrt((TECS.TRIM_THROTTLE.value - telemetry.throttle)
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
-- A cleared source field is NOT nil on Ethos: it hands back a real Source object
-- reporting name "---" and value 0. Every `== nil` test on a configured source
-- has to go through this or an unset field reads as a live one stuck at zero.
local function sourceOrNil(src)
  if src == nil then return nil end
  local ok, name = pcall(function() return src:name() end)
  if ok and (name == nil or name == "" or name == "---") then return nil end
  return src
end

local function advance(widget)
  -- every captured value now comes from telemetry, so advancing on a dead link
  -- would silently record zeros into the parameters. Refuse, and say why.
  if not telemetryOk then
    widget.warn = "no telemetry - step not recorded"
    return
  end
  widget.warn = nil
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
  local sw = sourceOrNil(widget.switchSource)
  if sw == nil then return end
  local now = os.clock()
  local active = sw:value() > 0
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
    backColor      = lcd.RGB(0, 0, 0),
    foreColor      = lcd.RGB(255, 255, 255),
    linkMode       = LINK_AUTO,  -- which telemetry transport to poll
    debugTelem     = false,      -- overlay the raw passthrough-frame diagnostics
    -- runtime state
    step           = 1,
    lastTrigger    = 0,
    switchWasActive = false,
    warn           = nil,        -- shown under "info:" when a step is refused
  }
end

local function wakeup(widget)
  local got = false

  -- Both transports are drained every wakeup. They are mutually exclusive on a
  -- given model (an SPort link produces no CRSF frames and vice versa), so
  -- whichever one is live wins and the widget needs no configuration to match
  -- the receiver.
  if widget.linkMode ~= LINK_CRSF then
    -- FrSky SPort/F.Port: drain waiting passthrough frames (R9 / X-S-series).
    -- 10 per wakeup matches Yaapu; more than that risks the CPU-usage killer.
    for _ = 1, 10 do
      if not sportPop() then break end
      got = true
    end
  end
  if widget.linkMode ~= LINK_SPORT then
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

-- greedily word-wraps `text` to fit `maxW` at the *current* font.
-- returns a list of lines (never empty for non-empty input).
local function wrapText(text, maxW)
  local lines = {}
  local line = ""
  for word in string.gmatch(text, "%S+") do
    local try = (line == "") and word or (line .. " " .. word)
    local tw = lcd.getTextSize(try)
    if tw > maxW and line ~= "" then
      lines[#lines + 1] = line
      line = word
    else
      line = try
    end
  end
  if line ~= "" then lines[#lines + 1] = line end
  return lines
end

-- Diagnostic page for both transports. Each SPort reader is counted separately
-- (dflt = no module key, mod1 = module=1) so the live binding is visible, and
-- CRSF has its own counters. The app-id list is fed from processTelemetry, so
-- it fills in for whichever transport is carrying the link.
local function paintDebug(widget, w, h)
  lcd.font(FONT_S)
  local _, lineH = lcd.getTextSize("0")
  local row = lineH + 2
  local y = 2

  lcd.drawText(4, y, "TELEM DEBUG"); y = y + row
  lcd.drawText(4, y, "readers: " .. (#sportReaders > 0
    and (#sportReaders .. " built") or (sportErr or "not built yet"))); y = y + row
  for _, r in ipairs(sportReaders) do
    lcd.drawText(4, y, string.format("  %s: pops %d  frames %d",
      r.name, r.pops, r.frames)); y = y + row
  end
  lcd.drawText(4, y, string.format("frames: %d", sportFrames)); y = y + row
  lcd.drawText(4, y, string.format("pops: %d  nondata: %d",
    sportPops, sportNonData)); y = y + row
  lcd.drawText(4, y, string.format("crsf pops: %d  frames: %d",
    crsfPops, crsfFrames)); y = y + row
  lcd.drawText(4, y, string.format("mode: %s  %s", modeName(telemetry.mode),
    telemetry.armed == 1 and "ARMED" or "disarmed")); y = y + row
  if sportPopErr ~= nil then
    for _, l in ipairs(wrapText("popFrame: " .. sportPopErr, w - 8)) do
      lcd.drawText(4, y, l); y = y + row
    end
  end

  y = y + 4
  lcd.drawText(4, y, "appId = value (last seen):"); y = y + row
  if #telemSeenOrder == 0 then
    lcd.drawText(4, y, "(nothing received)"); y = y + row
  else
    for _, key in ipairs(telemSeenOrder) do
      if y > h - row then break end
      lcd.drawText(4, y, string.format("%s = %08X", key, telemSeen[key])); y = y + row
    end
  end
end

local function paint(widget)
  local w, h = lcd.getWindowSize()

  -- background
  lcd.color(widget.backColor)
  lcd.drawFilledRectangle(0, 0, w, h)
  lcd.color(widget.foreColor)

  if widget.debugTelem then
    paintDebug(widget, w, h)
    return
  end

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
  kv(telX, telValX, y, "climb m/s",   fmt(dmsToMs(telemetry.vSpeed))); y = y + row
  -- always the FC's own commanded throttle: the stick is only a proxy for it and
  -- any curve, expo or mix makes the two diverge, so there is nothing to choose
  kv(telX, telValX, y, "throttle %",  fmt(telemetry.throttle)); y = y + row + 4

  lcd.drawText(telX, y, "info:"); y = y + row
  kv(telX, telValX, y, "telemetry", telemetryOk and "OK" or "--"); y = y + row
  kv(telX, telValX, y, "step",      string.format("%d/%d", widget.step, STEP_COUNT + 1)); y = y + row
  if sourceOrNil(widget.switchSource) == nil then
    lcd.drawText(telX, y, "set trigger switch!"); y = y + row
  elseif widget.warn ~= nil then
    lcd.drawText(telX, y, widget.warn); y = y + row
  end

  -- --- TECS params block (right) ---
  local py = 2
  for _, name in ipairs(TECS_ORDER) do
    lcd.drawText(colX, py, name)
    lcd.drawText(valX, py, fmtParam(name), RIGHT)
    py = py + row
  end

  -- --- current instruction (bottom, centered, wrapped) ---
  -- FONT_S is taller/wider than the old FONT_XS, so a long caption won't fit
  -- on one line (especially on narrow X18/X18S screens). Wrap it and stack the
  -- lines upward from the bottom edge so nothing gets clipped off-screen.
  local instruction
  if widget.step == 1 then
    instruction = "engage the trigger switch to start"
  else
    instruction = stepDef[widget.step - 1].text
  end
  -- pick the largest font the screen can carry: bigger on wide radios
  -- (X20S ~784px), a step down on narrow ones (X18/X18S ~ up to ~300px widget).
  local capFont = FONT_XS
  if w >= 700 then
    capFont = FONT_STD
  elseif w >= 400 then
    capFont = FONT_S
  end
  lcd.font(capFont)
  local _, lineH = lcd.getTextSize(instruction)
  local lines = wrapText(instruction, w - 8)
  local iy = h - (#lines * lineH)
  for i = 1, #lines do
    lcd.drawText(math.floor(w / 2), iy, lines[i], CENTERED)
    iy = iy + lineH
  end
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

  -- Auto polls both transports; the forced modes are only needed for debugging.
  line = form.addLine("Telemetry link")
  form.addChoiceField(line, nil,
    { { "Auto", LINK_AUTO }, { "CRSF (ELRS/TBS)", LINK_CRSF }, { "FrSky SPort (R9)", LINK_SPORT } },
    function() return widget.linkMode end,
    function(v) widget.linkMode = v end)

  -- shows which 0x50xx sensors were found and their raw values, over the display
  line = form.addLine("Debug telemetry")
  form.addBooleanField(line, nil,
    function() return widget.debugTelem end,
    function(v) widget.debugTelem = v end)
end

-- ================================================================
-- PERSISTENCE
-- ================================================================
local function read(widget)
  widget.switchSource   = storage.read("switchSource")
  widget.linkMode       = storage.read("linkMode")
  -- widgets saved by <=v0.3.1 only have the old "useSport" boolean; Auto covers
  -- both of its settings, so they all migrate there
  if widget.linkMode == nil then widget.linkMode = LINK_AUTO end
  widget.debugTelem     = storage.read("debugTelem") or false
end

local function write(widget)
  storage.write("switchSource", widget.switchSource)
  storage.write("linkMode", widget.linkMode)
  storage.write("debugTelem", widget.debugTelem)
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
