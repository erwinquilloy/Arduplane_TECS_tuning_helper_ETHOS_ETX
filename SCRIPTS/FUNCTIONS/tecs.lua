-- tecs tuning advisor v0.2.4

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


local step=1            -- init
local stepCount = 8     -- step/loop count
local f                 -- logfile handle
local exdelay = 250     -- this limits opentx to fire the script too often
local debug = true		-- for printing debug and raw messages

telemetry = {}			-- shared from SCRIPTS/TELEMETRY/tecstm.lua
telemetry.pitch = 0		-- [-90 +90] deg
telemetry.airspeed = 50	-- dm/s
telemetry.hSpeed = 50	-- dm/s
telemetry.vSpeed = 3	-- dm/s

local function iniTTA()
    step = 1
    extime=getTime()
end
 
local function KPH_to_CMs(KPH)      return string.format("%d", (KPH/0.036) )			end
local function DMS_to_MS(DMS)      	return string.format("%d", (DMS*0.1) )    			end
local function DMs_to_CMs(DMS)      return string.format("%d", (DMS*10) )    			end
local function DMs_to_KPH(DMS)      return string.format("%d", (DMS/10*3.6) )    		end
local function KPH_to_Ms(KPH)       return string.format("%d", (KPH/3.6) )      		end
-- throttle as the flight controller outputs it (0x5001 AP_STATUS) - the unit
-- TRIM_THROTTLE and THR_MAX are in. The TX stick is only a proxy (expo, curves
-- and mixes make the two differ); it is used only if no AP_STATUS frame arrived
-- in the last 2 s.
local getThrottlePct = function()
	if telemetry.throttleTime ~= nil and getTime() - telemetry.throttleTime < 200 then
		return telemetry.throttle
	end
	return math.floor((getValue("thr")+1024)/ 20.48)
end


-- these are the global TECS parameters
-- each defined as a function to export the raw value into a different unit + security margins

TECS = {
--1
    TRIM_THROTTLE   = { value = 0,  exporter = function(v) return(v) end }, 							-- raw: percent,  output: percent
    AIRSPEED_CRUISE   = { value = 0,  exporter = function(v) return( DMS_to_MS(v) ) end },				-- raw: dm/s output: m/s (AIRSPEED_CRUISE, 4.5+)
--2
    THR_MAX         = { value = 0,  exporter = function(v) return(v) end }, 							-- raw: percent,  output: percent
	AIRSPEED_MAX   = { value = 0,  exporter = function(v) return( DMS_to_MS(v * 0.95 ) ) end },   		-- raw: dm/s output: m/s * 0.95
--3
    TECS_PITCH_MAX  = { value = -4, exporter = function(v) return(math.abs(v + 4)) end },    			-- raw: deg	output: +deg 4deg
	TECS_CLMB_MAX   = { value = 0,  exporter = function(v) return (math.min(math.abs(0.1*v),10)) end }, -- raw: dm/s output: +m/s
    FBWB_CLIMB_RATE = { value = 0,  exporter = function(v) return (math.min(math.abs(0.1*v),10)) end }, -- raw: dm/s output: m/s
--4
    AIRSPEED_MIN   = { value = 0,  exporter = function(v) return( DMS_to_MS(v ) ) end },   			-- raw: dm/s , output: m/s
--5
    STAB_PITCH_DOWN = { value = 0,  exporter = function(v) return(math.abs(v)) end },   				-- deg
    TECS_SINK_MIN   = { value = 0,  exporter = function(v) return(math.min(math.abs(0.1*v),10)) end },  -- raw: dm/s , output: m/s
--6
    TECS_PITCH_MIN  = { value = 4,  exporter = function(v) return(v - 4) end },    						-- raw deg output: +/-deg - 4
    TECS_SINK_MAX   = { value = 0,  exporter = function(v) return(math.min(math.abs(0.1*v),10))  end }, -- raw: dm/s output: +m/s
--7
-- KFF_THR2PTCH (step 8) = pitch * 100 / throttle (Stavros' ArduPilot setup checklist).
-- ArduPlane adds KFF_THR2PTCH x throttle%/100 degrees counted from ZERO throttle, so
-- this is exact at the step-8 throttle, and at cruise it also adds
-- KFF_THR2PTCH x TRIM_THROTTLE/100 degrees. Clamped to ArduPlane's documented
-- range (-5..5); 0 if no throttle was recorded.
    KFF_THR2PTCH    = { value = 0,  exporter = function(v) return(math.floor(v * 100 + 0.5) / 100) end },
}

-- step 8 record (not a parameter): pitch (deg) and throttle (%) at full speed, raw KFF
FULLSPEED = { pitch = nil, thr = nil, kffRaw = 0 }



-- these are the tuning steps
-- each step has a audio and text description and can set multiple params
local stepDef = {
    step1 = {
        audio = function() 		playFile("tecs10.wav") end,
        text  = function(arg)	return "continue in Fly by Wire A and fly level at desired cruise speed" end,
        fn    = function(arg)
            TECS['TRIM_THROTTLE'].value = getThrottlePct()		--38 -- 
            TECS['AIRSPEED_CRUISE'].value = (telemetry.airspeed ~= 0 and telemetry.airspeed) or telemetry.hSpeed 		-- 150 -- 65kph
            return
        end,
    },
    step2 = {
        audio = function() 		
			playFile("tecs11.wav") 
			playNumber( DMs_to_KPH(TECS['AIRSPEED_CRUISE'].value), 7) ---- YOU ARE HERE, DO THIS FOR ALL OF THEM AND DISREGARD function call
			
			playFile("tecs20.wav")
		end,  
        text  = function(arg)   return "now accelerate to your desired maximum cruise speed" end, 
        fn    = function(arg)
            TECS['THR_MAX'].value 		= getThrottlePct()		-- 80 -- 
            TECS['AIRSPEED_MAX'].value = (telemetry.airspeed ~= 0 and telemetry.airspeed) or telemetry.hSpeed 		-- 230 -- "82kph"
            return
        end,
    },
    step3 = {
        audio = function() 
            playFile("tecs21.wav")
			playNumber( DMs_to_KPH(TECS['AIRSPEED_MAX'].value), 7)
			
            playFile("tecs30.wav") 
            playNumber( TECS['THR_MAX'].value, 13)
            playFile("tecs31.wav") 
            playNumber( DMs_to_KPH(TECS['AIRSPEED_CRUISE'].value), 7)
        end,
        text = function(arg)    return string.format("keep the throttle at %s and start climbing until your airspeed reaches %s kilometer per hour.", TECS['THR_MAX'].value, DMs_to_KPH(TECS['AIRSPEED_CRUISE'].value))    end,    
        fn   = function(arg)
            TECS['TECS_PITCH_MAX'].value 	= telemetry.pitch 		-- 27
            TECS['TECS_CLMB_MAX'].value 	= telemetry.vSpeed 		-- 70	--
            TECS['FBWB_CLIMB_RATE'].value 	= telemetry.vSpeed 		-- 70	--
            return
        end,
    },    
    step4 = {
        audio = function() 		
			playFile("tecs32.wav")
			playNumber( math.min(math.abs(0.1*TECS['TECS_CLMB_MAX'].value),10), 5)

			playFile("tecs40.wav") 
		end,
        text  = function(arg)   return "slow down to the minimum safe speed without stalling" end,
        fn    = function(arg)
            TECS['AIRSPEED_MIN'].value = (telemetry.airspeed ~= 0 and telemetry.airspeed) or telemetry.hSpeed 		-- 120 -- "46kph" 
            return
        end,
    },
    step5 = {
        audio = function() 
            playFile("tecs41.wav") 
			playNumber( DMs_to_KPH(TECS['AIRSPEED_MIN'].value) ,7)

            playFile("tecs50.wav") 
            playNumber( DMs_to_KPH(TECS['AIRSPEED_MIN'].value) ,7)
        end,
        text = function(arg)    return string.format("gain some altitude, then cut throttle and pitch down just enough to hold %s kph",DMs_to_KPH(TECS['AIRSPEED_MIN'].value))        end,
        fn   = function(arg)
            -- 5a: smallest nose-down that holds AIRSPEED_MIN with the throttle cut
            TECS['STAB_PITCH_DOWN'].value = telemetry.pitch 	-- "-3" -- 
            return
        end,
    },
    step6 = {
        -- 5b: TECS_SINK_MIN is the sink rate at THR_MIN and AIRSPEED_CRUISE (ArduPlane
        -- docs; TECS's throttle model pairs it with TECS_CLMB_MAX at the same speed)
        audio = function() 
            playFile("tecs50.wav") 
            playNumber( DMs_to_KPH(TECS['AIRSPEED_CRUISE'].value) ,7)
        end,
        text = function(arg)    return string.format("keep the throttle cut and pitch down until airspeed reaches %s kph",DMs_to_KPH(TECS['AIRSPEED_CRUISE'].value))        end,
        fn   = function(arg)
            TECS['TECS_SINK_MIN'].value =  	telemetry.vSpeed 	-- 20 --
            return
        end,
    },
    step7 = {
        audio = function() 
            playFile("tecs51.wav") 
			playNumber( math.min(math.abs(0.1*TECS['TECS_SINK_MIN'].value),10), 5)

            playFile("tecs60.wav") 
            playNumber( DMs_to_KPH(TECS['AIRSPEED_MAX'].value) ,7)
        end,
        text = function(arg)    return string.format("continue with zero throttle and pitch down until airspeed reaches %s kph",DMs_to_KPH(TECS['AIRSPEED_MAX'].value))        end,
        fn   = function(arg)
            TECS['TECS_PITCH_MIN'].value =	telemetry.pitch 		-- "-24" --
            TECS['TECS_SINK_MAX'].value  =	telemetry.vSpeed 		-- 120 -- 
            return
        end
    },
    step8 = {
        audio = function() 		
			playFile("tecs61.wav") 
			playNumber( math.min(math.abs(0.1*TECS['TECS_SINK_MAX'].value),10), 5)

			playFile("tecs70.wav") 
		end,
        text  = function(arg)   return string.format("fly at your step 2 throttle (THR_MAX) and hold altitude")        end,
        fn    = function(arg)
            FULLSPEED.pitch = telemetry.pitch
            FULLSPEED.thr   = getThrottlePct()
            -- Stavros: KFF_THR2PTCH = pitch * 100 / throttle (see the note at TECS above)
            local kff = 0
            if FULLSPEED.thr ~= nil and FULLSPEED.thr > 0 then
                kff = FULLSPEED.pitch * 100 / FULLSPEED.thr
            end
            FULLSPEED.kffRaw = kff
            TECS['KFF_THR2PTCH'].value = math.max(-5, math.min(5, kff))
            return
        end
    },
}

local function logTECS(TECS)
	local datenow = getDateTime()
	local timestamp = datenow.year..""..string.format("%02d",datenow.mon)..""..string.format("%02d",datenow.day)..'_'..string.format("%02d",datenow.hour)..""..string.format("%02d",datenow.min)
	local f = io.open("/LOGS/tecs_"..timestamp..".txt", "a")
	
	for param in next,TECS,nil do 
		local exportValue = nil
		while not exportValue do
			exportValue = TECS[param].exporter(TECS[param].value)
			if type(exportValue) == "lightfunction" then        -- https://github.com/opentx/opentx/issues/6201
				exportValue = nil
			end
		end
		io.write(f, string.format("%s=%s\r\n", param, exportValue ))
	end
	if FULLSPEED.pitch ~= nil and FULLSPEED.thr ~= nil and FULLSPEED.thr > 0 then
		local kff = TECS['KFF_THR2PTCH'].value
		io.write(f, string.format("# KFF_THR2PTCH = pitch*100/throttle (Stavros): step 8 pitch %.1f deg at %d%% throttle\r\n", FULLSPEED.pitch, FULLSPEED.thr))
		io.write(f, string.format("#   gives %.1f deg at %d%% and %.1f deg at %d%% cruise (TRIM_THROTTLE)\r\n", kff * FULLSPEED.thr / 100, FULLSPEED.thr, kff * TECS['TRIM_THROTTLE'].value / 100, TECS['TRIM_THROTTLE'].value))
		if math.abs(FULLSPEED.kffRaw) > 5 then
			io.write(f, string.format("#   clamped to the ArduPlane range +/-5 (calculated %.2f)\r\n", FULLSPEED.kffRaw))
		end
	else
		io.write(f, "# KFF_THR2PTCH: step 8 not recorded - left at 0\r\n")
	end
	
	if debug then
		for param in next,TECS,nil do 
			local exportValue = nil
			while not exportValue do
				exportValue = TECS[param].exporter(TECS[param].value)
				if type(exportValue) == "lightfunction" then        -- https://github.com/opentx/opentx/issues/6201
					exportValue = nil
				end
			end
			io.write(f, string.format("debug_%s=%s\r\n", param, TECS[param].value ))
		end
	end
	io.close(f)
end



-- this gets executed each time the switch is been triggered
-- runs each function, text or audio to the related step
-- saves everything to a logfile called "tecs.txt"
local function runTTA()
    if extime+exdelay < getTime() then

         if step == 1 then
             stepDef["step"..step]["audio"]()                                    -- play audio
             step=step+1
         else
             local prevStep=step-1
             stepDef["step"..prevStep]["fn"]()                                   -- execute function of previous step
 
             if step > stepCount then                                            -- reset and print summary
                 step = 1
				 logTECS(TECS)
                 playFile("tecsf.wav")
 
             else
                 stepDef["step"..step]["audio"]()                                -- play audio instructions
                 step=step+1
             end
         end
        extime=getTime()
    end
    return 1
end
 
return { init=iniTTA, run=runTTA }
 
