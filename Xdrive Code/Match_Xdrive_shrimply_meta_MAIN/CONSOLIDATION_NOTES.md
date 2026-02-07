# Match_Xdrive_shrimply_meta_MAIN - Consolidation Summary

## Overview
This is a unified robot code folder created by consolidating 4 separate position-specific robot configurations into a single codebase that supports all match play scenarios.

## What Was Done

### Original 4 Folders (Now Deprecated)
- **LeftsideBlue_Xdrive_shrimply_meta_MAIN** - Blue alliance, left side
- **RedLeftside_Xdrive_shrimply_meta_MAIN** - Red alliance, left side
- **RedRightside_Xdrive_simply_meta_main** - Red alliance, right side
- **RightsideBlue_Xdrive_shrimply_meta_MAIN** - Blue alliance, right side (had selector framework)

### New Unified Folder Structure
- **Match_Xdrive_shrimply_meta_MAIN/** - Single source of truth for all match autonomous routines

## Key Features

### 1. Auton Selector System (3-Step UI)
Located in `src/main.cpp` starting ~line 365:
- **Step 1**: Select Alliance Color (RED or BLUE)
- **Step 2**: Select Side (RIGHT or LEFT)
- **Step 3**: Select Goal Type (CENTER or LONG scoring)
- **Result**: Maps to 1 of 8 autonomous routines automatically

### 2. All 8 Autonomous Routines Implemented
```
BLUE ALLIANCE:
- BlueLeftCenter()   - Left side, center goal scoring
- BlueLeftLong()     - Left side, long-range scoring
- BlueRightCenter()  - Right side, center goal scoring
- BlueRightLong()    - Right side, long-range scoring

RED ALLIANCE:
- RedLeftCenter()    - Left side, center goal scoring
- RedLeftLong()      - Left side, long-range scoring
- RedRightCenter()   - Right side, center goal scoring
- RedRightLong()     - Right side, long-range scoring
```

### 3. Standardized Color Sensor Logic
**Unified in `colorsensor()` function (~line 66):**
- RED detected → Eject at 80% forward
- BLUE detected → Eject at 80% reverse
- Consistent across all alliances

### 4. Core Motion Functions (with Gyro Correction)
- `driveForwardInches()` - Tank drive with heading correction
- `turnToAngle()` - Gyro-based rotation
- `strafeRightInches()` - X-drive strafing with heading hold
- `backToWallSlow()` - Gradual wall approach

### 5. Intake & Scoring Functions
- `Intake()` - Full intake spin
- `colorintake()` - Intake with color sensor enabled
- `Bottomscore()` - Lower basket scoring
- `Middlescore()` - Middle basket scoring
- `Topscore()` - Top basket scoring
- `Dejam()` - Intake unjam utility

### 6. User Control
Standard button mapping implemented for manual control during driver-controlled period.

## Code Structure

### Pre-Autonomous (`pre_auton()`)
1. Calls `autonSelector()` for 3-step UI selection
2. Calibrates gyro
3. Sets deloader pneumatic position

### Autonomous (`autonomous()`)
1. Calls `switchMatchAuton()` 
2. Executes selected routine based on `autonToRun` variable
3. All 8 cases fully populated with corresponding functions

### Driver Control (`usercontrol()`)
- Motor diagnostics on brain display
- Color sensor monitoring during intake
- Button controls for all mechanisms

## Changes from Original Code

### Consolidated From Multiple Sources
| Feature | Source |
|---------|--------|
| Auton Selector UI | RightsideBlue |
| BlueLeft routines | LeftsideBlue |
| RedLeft routines | RedLeftside |
| RedRight routines | RedRightside |
| BlueRight routines | RightsideBlue |

### Standardizations Made
1. **Color Sensor**: All use same logic (80% eject speeds)
2. **Motion Parameters**: Unified FORWARD_SPEED=30, TURN_SPEED=30, SCORING_SPEED=45
3. **Gyro Correction**: Consistent GYRO_KP=2.0 across all routines
4. **Function Names**: Consistent naming (e.g., `BlueLeftCenter()` not `AutonScoringOne()`)

## Testing Checklist

- [ ] All 8 autonomous routines execute without errors
- [ ] Color sensor sorting works in all 8 configurations
- [ ] Gyro heading correction maintains heading during strafes
- [ ] Button controls responsive in driver control
- [ ] Motor temperature/current display functional
- [ ] Selector UI responsive and maps correctly to routines

## Files Included

```
Match_Xdrive_shrimply_meta_MAIN/
├── src/main.cpp          (28KB - unified codebase with all routines)
├── include/vex.h         (VEX SDK header)
├── makefile              (Build rules)
├── .gitignore            (Git ignore patterns)
├── vex/                  (Build environment)
│   ├── mkenv.mk
│   └── mkrules.mk
├── .vscode/              (VSCode settings)
│   ├── c_cpp_properties.json
│   ├── extensions.json
│   ├── settings.json
│   └── vex_project_settings.json
└── build/                (Compiled output directory)
```

## Usage

1. **Deploy to Robot**: Use VEXcode or your build system to compile and upload to robot
2. **Run Pre-Auton**: Robot enters selector UI automatically
3. **Select Configuration**: Use brain touchscreen to pick color → side → goal
4. **Run Match**: Autonomous executes selected routine
5. **Driver Control**: Manual control with standard button mapping

## Notes for Future Work

- All 8 routines have been extracted and are **ready for fine-tuning**
- Color sensor thresholds (RED_VAL=20, BLUE_VAL1=120, BLUE_VAL2=230) may need adjustment based on actual game field lighting
- Motion parameters (speeds, distances, angles) are tuned for this robot configuration - adjust if mechanisms change
- Special functions from RedLeftside (parking, deloading cycles) are available but not currently used in main routines

## Previous Deprecated Folders

The original 4 position-specific folders have been deleted in commit `194c7f0` and are no longer needed. This unified folder is the single source of truth going forward.

---

**Created**: February 7, 2026  
**Status**: Ready for robot deployment and testing
