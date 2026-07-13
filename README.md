# Arduplane TECS tuning helper — Ethos / EdgeTX

> A fork of **[mf0o](https://github.com/mf0o)**'s original
> [opentx_arduplane_tecs_tuning_helper](https://github.com/mf0o/opentx_arduplane_tecs_tuning_helper),
> extended with a FrSky **Ethos** port and updated for ArduPlane 4.5+ parameter
> names. All the original design and workflow are mf0o's — full credit to them.

### Description
This LUA script/widget will navigate you through the steps to tune your plane TECS.
This fork is focused on FrSky **Ethos** (X20S, X18/X18S) and **EdgeTX** radios
(RadioMaster TX16S mk2/mk3, Horus X10/X12, Taranis X9D/Q7).
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
* CRSF passthrough (Crossfire & ELRS) **and** FrSky SPort passthrough
  (R9, X/S-series receivers, F.Port). SPort is fully wired on EdgeTX/OpenTX; the
  Ethos widget has a SPort toggle too but it is **untested on hardware**
* read airplane telemetry
* step-by-step instructions for tuning the TECS
* single switch operation
* show TECS parameter in Arduplane format and unit on screen
* write logfiles to /LOGS/tecs_\<timestamp\>.txt

### Installation Ethos (X20S / X20 / X18 / X18S / Twin ...)
The Ethos version is a single self-contained widget that adapts to the radio's
screen size (verified layout logic for both 800x480 and 480x320/272 displays).
It reads ArduPilot **CRSF passthrough** telemetry (Crossfire & ELRS), same as the
EdgeTX version.

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

Requirements on the aircraft side are identical to the EdgeTX version
(ArduPilot frsky passthrough over CRSF). Logfiles are written to
`/scripts/tecs/tecs_<timestamp>.txt`.

### Installation EdgeTX color radios (RadioMaster TX16S mk2 & mk3, Horus X10/X12, ...)
This is the EdgeTX widget. It runs unchanged on any 480x272 color widget
radio, including the RadioMaster **TX16S mk2 and mk3** (both run EdgeTX with the
same Lua widget API — no separate build is needed for either revision).

* copy `WIDGETS/TECS/main.lua` to your SD card
* Choose your prefered voice and copy the custom sounds from `SOUNDS/\<voice-of-your-choice\>.zip` to your SD-card `/SOUNDS/en/`
* unload/remove yaapu Telemetry Script temporarily from the active Widgets List
* load the TECS widget as fullscreen widget
![](_img/horus_setup.png)
* [optionally] enter "widget settings" and choose your switch to initiate the next step. Default is SH
![](_img/horus_settings.png)
* in "widget settings" set **UseCRSF**: leave it **ON** for Crossfire/ELRS links,
  or turn it **OFF** for FrSky SPort links (R9, X/S-series receivers, F.Port).
  See [Telemetry link type](#telemetry-link-type-crsf-vs-frsky-sport) below.

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

> **FrSky SPort link (R9 etc.):** this telemetry-script version has no settings
> menu, so open `SCRIPTS/TELEMETRY/tecsX9.lua` and set `enableCRSF = false` in the
> `conf` table near the top before copying it to the SD-card. See below.

### Telemetry link type (CRSF vs FrSky SPort)

The widget/script only cares about how ArduPilot's **passthrough** telemetry
reaches the radio, not the RF brand. The same `0x50xx` passthrough app-ids are
carried by both transports, so both work:

| Link | Examples | Radio transport |
|------|----------|-----------------|
| **CRSF passthrough** | TBS Crossfire, ELRS | `crossfireTelemetryPop()` |
| **FrSky SPort passthrough** | **FrSky R9**, X/S-series Rx, F.Port | `sportTelemetryPop()` |

**Aircraft side (ArduPilot):**
* CRSF: serial port set to `SERIALx_PROTOCOL = 23` (RCIN/CRSF) with passthrough, as before.
* FrSky SPort: set the port wired to the receiver's **SmartPort** pad to
  `SERIALx_PROTOCOL = 10` (FrSky SPort passthrough), `SERIALx_BAUD = 57`. R9 /
  X-series SmartPort is inverted — use the flight-controller's inverted SPort pad
  (or an uninverted F.Port pad). Then **Discover new sensors** once on the radio's
  telemetry page so the `0x50xx` sensors appear.

**Radio side:**
* **EdgeTX color widget** (`WIDGETS/TECS/main.lua`): toggle **UseCRSF** in the
  widget settings — **ON** = CRSF, **OFF** = FrSky SPort.
* **X9D/Q7 telemetry script** (`SCRIPTS/TELEMETRY/tecsX9.lua`): edit
  `enableCRSF = true` → `false` in the `conf` table (no in-app menu).
* **Ethos** (X20S/X18…): long-press the widget → **Configure** → turn on
  **"FrSky SPort link (R9)"**. Ethos has no raw SPort frame API, so this path
  instead reads the discovered `0x50xx` passthrough **sensors** (make sure they
  show up under **Discover new sensors** first). **This Ethos SPort path is
  untested on hardware** — verify Pitch/Roll track the airframe before relying
  on it, and please report back if it works.

### Preparation (before you tune)

Adapted from Stavros' [Tuning the TECS](https://notes.stavros.io/ardupilot/tuning-the-tecs/)
notes — do these **before** the tuning flights so your measurements are
accurate. Parameter names are ArduPlane 4.5+; older names are noted in
parentheses.

* **Finish a successful autotune first.** TECS tuning assumes the roll/pitch
  controllers are already tuned.
* **Un-clamp the pitch limits** so they don't constrain the aircraft while you
  measure climb and descent:
    * `PTCH_LIM_MAX_DEG = 45`
    * `PTCH_LIM_MIN_DEG = -45`
    * *(ArduPlane ≤4.4 called these `LIM_PITCH_MAX = 4500` /
      `LIM_PITCH_MIN = -4500`, in centidegrees.)*
* **Get a raw (non-remapped) throttle reading** while measuring: set
  `THR_PASS_STAB = 1`.
* **Enable throttle battery-voltage compensation** so a partly-drained pack
  doesn't make the motor — and therefore your measurements — run slow:
    * `FWD_BAT_VOLT_MAX = 4.2 * cells` (Li-Ion or LiPo)
    * `FWD_BAT_VOLT_MIN = 3.0 * cells` (Li-Ion) or `3.5 * cells` (LiPo)
* **Have pitch and airspeed visible.** This widget already shows both on the
  telemetry screen; adding them to your OSD helps you cross-check in the air.

> Several of these (the pitch limits especially) change how the airframe flies.
> Note your original values first so you can restore them once tuning is done.

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
*  a logfile is written to your radio's SD card:
	*  **EdgeTX** (TX16S etc.): `/LOGS/tecs_<timestamp>.txt`
	*  **Ethos** (X20S etc.): `/scripts/tecs/tecs_<timestamp>.txt`

### Post-tuning — applying the values

The script **only measures and records** the values — it does **not** talk to the
aircraft. Nothing on the plane changes until *you* write the parameters with a
ground station. Do this on the bench after you've landed, not in the air.

#### 1. Land and recover the log

Land the plane and disarm before you start. The values live in two places:

* on the **TECS telemetry screen** (stays filled until the script is reloaded or the radio is restarted), and
* in the **log file** written when the sequence finishes:
	* **EdgeTX** (TX16S etc.): `/LOGS/tecs_<timestamp>.txt`
	* **Ethos** (X20S etc.): `/scripts/tecs/tecs_<timestamp>.txt`

The log file is plain text, one parameter per line as `NAME=value`, e.g.:

```
TRIM_THROTTLE=38
AIRSPEED_CRUISE=18
THR_MAX=80
...
debug_TRIM_THROTTLE=38   <- raw captured value, for troubleshooting only
```

Ignore the `debug_*` lines when applying — the plain `NAME=value` lines are the
ones you set. On Ethos, if `os.date` isn't available on your build the
`<timestamp>` is a plain counter number instead of a date, so sort by
modified-time to find the newest file.

#### 2. Get the log file to your computer

* Connect the radio in **USB storage / drive mode** (EdgeTX: *SYS → hold → USB*; Ethos: *plug in and choose the storage option*), **or** pull the SD card and read it directly.
* Copy the newest `tecs_<timestamp>.txt` to your PC.

#### 3. Write the parameters — pick one

1. **Type them in by hand.** Open *Config → Full Parameter List* in MissionPlanner (or the equivalent in QGroundControl / [Parachute](https://gitlab.com/stavros/parachute)), find each parameter below, enter the value from the screen or log, then **Write Params**. Fine if you only have a handful.
2. **Merge the log in Mission Planner (recommended).** In *Config → Full Parameter List* use **Load from file** (or **Compare Params**) and point it at `tecs_<timestamp>.txt`. Review the diff it shows, then **Write Params**. Fastest and least error-prone for the full set. *(The file's `NAME=value` layout matches the param format; if your MP version is strict about it, paste the values into the matching rows manually.)*
3. **Copy from the screen.** No computer? Read the values straight off the TECS telemetry screen and enter them in your ground station over a telemetry/Bluetooth link.

The parameters this tool sets (13 total):

| Group | Parameters |
|---|---|
| Throttle / speed | `TRIM_THROTTLE`, `THR_MAX`, `AIRSPEED_CRUISE`, `AIRSPEED_MIN`, `AIRSPEED_MAX` |
| Climb | `TECS_PITCH_MAX`, `TECS_CLMB_MAX`, `FBWB_CLIMB_RATE` |
| Sink / descent | `TECS_PITCH_MIN`, `TECS_SINK_MIN`, `TECS_SINK_MAX`, `STAB_PITCH_DOWN` |
| Feed-forward | `KFF_THR2PTCH` |

#### 4. Review, write, and verify

* **Compare against the originals you noted before tuning** (see the prep section). The pitch/climb/sink limits especially change how the airframe flies — sanity-check anything that looks extreme.
* **Write Params**, then **refresh/read them back** to confirm they stuck.
* Reboot the flight controller if your ground station recommends it for the changed params.
* Test conservatively on the next flight — keep altitude and be ready to switch back to a manual mode.

> Re-running the sequence overwrites the on-screen values in place; it does **not**
> reset them. A partial re-run therefore leaves earlier parameters at their
> previous readings, and the next log file will contain that mix — finish a full
> run before trusting a log.


### detailed Process
![](_img/tecs_tuning_process.png)

##### additional Resources
[https://ardupilot.org/plane/docs/tecs-total-energy-control-system-for-speed-height-tuning-guide.html
](https://ardupilot.org/plane/docs/tecs-total-energy-control-system-for-speed-height-tuning-guide.html)[https://notes.stavros.io/ardupilot/tuning-the-tecs/](https://notes.stavros.io/ardupilot/tuning-the-tecs/)
[https://notes.stavros.io/ardupilot/tecs-tuning-calculator/](https://notes.stavros.io/ardupilot/tecs-tuning-calculator/)

###### ... with many thanks to:
* [https://github.com/mf0o](https://github.com/mf0o) — original author of this project
* [https://github.com/shellixyz](https://github.com/shellixyz)
* [https://github.com/yaapu](https://github.com/yaapu)
* [https://github.com/skorokithakis](https://github.com/skorokithakis)


> Disclaimer:
> 
> Use on own risk without any warranty!

