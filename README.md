# Arduplane TECS tuning helper — Ethos / EdgeTX / OpenTX

### Description
This LUA script/widget will navigate you through the steps to tune your plane TECS.
It runs on FrSky **Ethos** (X20S, X18/X18S), and on **EdgeTX/OpenTX** color and
b/w radios (RadioMaster TX16S mk2/mk3, Horus X10/X12, Taranis X9D/Q7).
The data will be processed and reformatted to Ardupilot parameter units (i.e. dm/s->kph) on the Transmitter directly.
Finally the TECS will be displayed on the screen and saved to a logfile.

This is based on Yaapu's [FrSky Telemetry Script](https://github.com/yaapu/FrskyTelemetryScript/). 
Before you continue, make sure you have everything set up on Arduplane and your RC Link to have this working (passthrough telemetry).

It requires some custom sounds but still uses the built-in numbers and units on callouts.
The sound files are provided in /SOUNDS/ but matching, complete soundpacks (including the TECS sounds) can be found here: [OpenTX_soundpacks](https://github.com/mf0o/OpenTX_soundpacks)
A description of filenames and text are located in assets/custom_sounds.csv

The script is running in a loop of:

* read instructions of next step
* wait for you to get the plane in the desired state, attitude or speed
* save the related attributes

Each step is triggered by the configured switch and will update your TECS, which are displayed on the telemetry screen.

The process can not be paused or aborted but repeated as many times as needed.

### Summary
* CRSF protocol only as of today (crossfire & ELRS)
* read airplane telemetry
* step-by-step instructions for tuning the TECS
* single switch operation
* show TECS parameter in Arduplane format and unit on screen
* write logfiles to /LOGS/tecs_\<timestamp\>.txt

### Installation Ethos (X20S / X20 / X18 / X18S / Twin ...)
The Ethos version is a single self-contained widget that adapts to the radio's
screen size (verified layout logic for both 800x480 and 480x320/272 displays).
It reads ArduPilot **CRSF passthrough** telemetry (Crossfire & ELRS), same as the
OpenTX version.

* copy the whole `ETHOS/scripts/tecs/` folder to your SD-card `/scripts/tecs/`
* choose a voice and extract the `tecs*.wav` files from `SOUNDS/<voice>.zip`
  into `/scripts/tecs/audio/en/` (see `PLACE_WAVS_HERE.txt` there)
* reboot the radio, then add a screen/widget and pick **"TECS Tuning"**
  as a full-screen widget
* long-press the widget → **Configure**:
    * **Trigger switch** – the switch (momentary recommended) that advances the steps
    * **Throttle source** – *(optional)* your throttle stick/channel; if left empty the
      widget uses the throttle value reported by ArduPilot's VFR telemetry
* open the screen and confirm Pitch/Roll update when you move the aircraft

Requirements on the aircraft side are identical to the OpenTX version
(ArduPilot frsky passthrough over CRSF). Logfiles are written to
`/scripts/tecs/tecs_<timestamp>.txt`.

### Installation Horus / EdgeTX color radios (RadioMaster TX16S mk2 & mk3, Horus X10/X12, ...)
This is the OpenTX/EdgeTX widget. It runs unchanged on any 480x272 color widget
radio, including the RadioMaster **TX16S mk2 and mk3** (both run EdgeTX with the
same Lua widget API — no separate build is needed for either revision).

* copy `WIDGETS/TECS/main.lua` to your SD card
* Choose your prefered voice and copy the custom sounds from `SOUNDS/\<voice-of-your-choice\>.zip` to your SD-card `/SOUNDS/en/`
* unload/remove yaapu Telemetry Script temporarily from the active Widgets List
* load the TECS widget as fullscreen widget
![](_img/horus_setup.png)
* [optionally] enter "widget settings" and choose your switch to initiate the next step. Default is SH
![](_img/horus_settings.png)

* reboot your radio to flush the widget cache
* telemetry values on the left should change when moving your aircraft
![](_img/horus_example.png)


### Installation x9D /Q7 etc.
* Copy `SCRIPTS/FUNCTIONS/tecs.lua` to your SD-card `/SCRIPTS/FUNCTIONS/`
* Copy `/SCRIPTS/TELEMETRY/tecsX[7|9].lua` to your SD-card `/SCRIPTS/TELEMETRY/`
* Choose your prefered voice and copy the custom sounds from `SOUNDS/\<voice-of-your-choice\>.zip` to your SD-card `/SOUNDS/en/`
* replace your yaapu* from the models telemetry screen with "tecstm" 
![](_img/telemetry_screen_tecstm.png)
	* (Although the script is based on Yaapu FrSky Telemetry Script 1.9.5, it cant be used simulatenously)
* set up a switch in SPECIAL FUNCTION to trigger the script (momentary switch recommended)
![](_img/special_functions.png)
* open your telemetry screen and validate that Pitch and Roll updating accordingly to aircraft movement
![](_img/telemetry_screen_empty.png)

### Operation

**! You are 100% of the time in control and responsible for your plane !**

*There is no need to do any risky manouvers, you can abort at any time and re-gain altitude etc. or cycle through the menu and start over again*

* open the telemetry screen on your remote, it will have 0s in all parameters
* launch your plane and climb to a comfortable altitude, continue in FBWA
* engage your switch
	* 	follow the instructions to get _and_ hold the plane in the requested attitude and/or speed [**give the telemetry here 1 or 2 second to update**]
	*  engage the switch again to save the values
	*  repeat
*  Once finished your TECS screen should be filled with numbers
*  a logfile will be written to /LOGS/tecs_\<timestamp\>.txt
*  Use MissionPlanner, QGroundControl or [Parachute](https://gitlab.com/stavros/parachute) to update your configuration


### detailed Process
![](_img/tecs_tuning_process.png)

##### additional Resources
[https://ardupilot.org/plane/docs/tecs-total-energy-control-system-for-speed-height-tuning-guide.html
](https://ardupilot.org/plane/docs/tecs-total-energy-control-system-for-speed-height-tuning-guide.html)[https://notes.stavros.io/ardupilot/tuning-the-tecs/](https://notes.stavros.io/ardupilot/tuning-the-tecs/)
[https://notes.stavros.io/ardupilot/tecs-tuning-calculator/](https://notes.stavros.io/ardupilot/tecs-tuning-calculator/)

###### ... with many thanks to:
* [https://github.com/shellixyz](https://github.com/shellixyz)
* [https://github.com/yaapu](https://github.com/yaapu)
* [https://github.com/skorokithakis](https://github.com/skorokithakis)


> Disclaimer:
> 
> Use on own risk without any warranty!

