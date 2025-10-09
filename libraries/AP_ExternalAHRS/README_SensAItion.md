# SensAItion Driver Test Guide

## Build Instructions

### 1. Build for SITL (with External AHRS support)
```bash
# Configure with external AHRS support enabled
./waf configure --board sitl \
    --enable-AHRS_EXT \
    --enable-EXTERNALAHRS_COMPASS \
    --enable-EXTERNALAHRS_BARO

# Build ArduPlane (automated tests are for Plane)
./waf build --target bin/arduplane

# Verify no C++ STL symbols (ArduPilot coding standard)
nm build/sitl/libraries/AP_ExternalAHRS/*.o | grep -i "std::" | grep -i sensaition
# Should return nothing!
```

### 2. Build for CubeOrangePlus (with External AHRS flags)
```bash
# Configure with external AHRS support enabled
./waf configure --board CubeOrangePlus \
    --enable-AHRS_EXT \
    --enable-EXTERNALAHRS_COMPASS \
    --enable-EXTERNALAHRS_BARO \
    --define AP_HEATER_IMU_INSTANCE=1

# Build (can be arduplane or arducopter depending on vehicle)
./waf build --target bin/arducopter

# Upload to hardware (optional)
./waf build --target bin/arducopter --upload
```

**Note:** The `--define AP_HEATER_IMU_INSTANCE=1` flag is required when using external AHRS.
By default, the IMU heater reads temperature from IMU 0. When SensAItion becomes IMU 0
(as external AHRS), the heater must read from IMU 1 (internal IMU) instead to avoid
"heater temp low" prearm failures.

## Unit Tests

### Run SensAItion Parser Tests
```bash
# Configure for SITL with external AHRS enabled
./waf configure --board sitl \
    --enable-AHRS_EXT \
    --enable-EXTERNALAHRS_COMPASS \
    --enable-EXTERNALAHRS_BARO

# Build and run all tests (includes SensAItion parser tests)
./waf tests

# Run only SensAItion parser test
./build/sitl/tests/test_sensaition_parser
```

Expected output:
```
[==========] Running 7 tests from 1 test suite.
[----------] 7 tests from SensAItionParser
[ RUN      ] SensAItionParser.ValidIMUPacket
[       OK ] SensAItionParser.ValidIMUPacket
[ RUN      ] SensAItionParser.InvalidChecksumIMU
[       OK ] SensAItionParser.InvalidChecksumIMU
[ RUN      ] SensAItionParser.ByteByByteParsing
[       OK ] SensAItionParser.ByteByByteParsing
[ RUN      ] SensAItionParser.NoiseBeforeHeader
[       OK ] SensAItionParser.NoiseBeforeHeader
[ RUN      ] SensAItionParser.MixedValidInvalidPackets
[       OK ] SensAItionParser.MixedValidInvalidPackets
[ RUN      ] SensAItionParser.ValidAHRSPacket
[       OK ] SensAItionParser.ValidAHRSPacket
[ RUN      ] SensAItionParser.InvalidChecksumAHRS
[       OK ] SensAItionParser.InvalidChecksumAHRS
[  PASSED  ] 7 tests.
```

## SITL Testing

**IMPORTANT:** Before running SITL tests, you MUST configure with external AHRS flags:
```bash
./waf configure --board sitl --enable-AHRS_EXT --enable-EXTERNALAHRS_COMPASS --enable-EXTERNALAHRS_BARO
./waf build --target bin/arduplane
```

### How SITL Tests Work

The SITL (Software In The Loop) simulator tests the SensAItion driver without hardware:

1. **Flight dynamics simulation** - JSBSim physics engine provides vehicle motion data (acceleration, rotation, magnetic field, altitude)
2. **Data conversion** - SIM_SensAItion.cpp converts SI units to SensAItion protocol units:
   - m/s² → µg (microG, 1e-6 scale)
   - rad/s → µdeg/s (microdegrees/sec, 1e-6 scale)
   - Gauss → mGauss (milliGauss)
   - Pa → 0.1 Pa units (barometer)
   - Euler angles → quaternion (for AHRS mode)
3. **Packet generation** - Binary packets sent via virtual serial port at configured rate (1000Hz IMU or 500Hz AHRS)
4. **Driver parsing** - AP_ExternalAHRS_SensAItion parses packets and feeds data to INS/EKF3
5. **Test verification** - Autotest checks EKF convergence and vehicle arming success

### IMU-only Mode Test (1000Hz)
The automated test runs IMU-only mode with EKF3:

```bash
# Run automated SensAItion IMU test
python3 Tools/autotest/autotest.py test.Plane.SensAItion
```

This test configures:
- EAHRS_TYPE = 11 (SensAItion)
- EAHRS_RATE = 1000 (1000Hz IMU packets)
- EAHRS_SENSORS = 14 (IMU=2 + Baro=4 + Compass=8)
- AHRS_EKF_TYPE = 3 (EKF3 uses external IMU)
- SERIAL4_BAUD = 460800

### AHRS Mode Test (500Hz)
Test with quaternion output:

```bash
# Run automated SensAItion AHRS test
python3 Tools/autotest/autotest.py test.Plane.SensAItionEAHRS
```

This test configures:
- EAHRS_TYPE = 11 (SensAItion)
- EAHRS_SENSORS = 14 (IMU=2 + Baro=4 + Compass=8)
- AHRS_EKF_TYPE = 11 (External AHRS passthrough)
- SERIAL4_BAUD = 460800

### Manual SITL Test
```bash
# Start SITL with SensAItion simulator (Plane or Copter)
sim_vehicle.py -v Plane -w --console --map -A "--serial4=sim:SensAItion"

# In MAVProxy, set parameters:
param set EAHRS_TYPE 11
param set SERIAL4_PROTOCOL 36
param set SERIAL4_BAUD 460800
param set AHRS_EKF_TYPE 3
param set EAHRS_SENSORS 14
param set EAHRS_RATE 1000

# Reboot and test
reboot
```

## Hardware Testing (CubeOrangePlus)

### Hardware Setup
- Connect SensAItion to Telem2 port (SERIAL2)
- Check SensAItion baud rate (typically 460800 or 921600)
- Power both devices

### Configure Parameters (via Mission Planner or MAVProxy)
```
EAHRS_TYPE       11        # SensAItion driver
SERIAL2_PROTOCOL 36        # External AHRS on Telem2
SERIAL2_BAUD     460       # 460800 baud
AHRS_EKF_TYPE    3         # EKF3 (for IMU mode) or 11 (for AHRS mode)
EAHRS_SENSORS    14        # IMU(2) + Baro(4) + Compass(8) = 14
```

### Verify Operation
1. **Check Messages tab** for "SensAItion ExternalAHRS initialised"
2. **Monitor attitude** - should update smoothly at high rate
3. **Check TEMP_DEBUG messages** (if debug enabled) showing packet rate
4. **No error messages** about ExternalAHRS unhealthy

### Troubleshooting
- **No data?** Check TX/RX connections, verify baud rate
- **Parse errors?** Wrong baud rate - check GCS messages for baud rate warning
- **Prearm fail?** Check EAHRS_SENSORS matches your configuration
- **Wrong attitude?** Verify sensor orientation/mounting

## Code Style Check
```bash
# Format SensAItion driver files
./Tools/CodeStyle/ardupilot-astyle.sh libraries/AP_ExternalAHRS/AP_ExternalAHRS_SensAItion.*

# Format SensAItion parser files
./Tools/CodeStyle/ardupilot-astyle.sh libraries/AP_ExternalAHRS/AP_ExternalAHRS_SensAItion_Parser.*
```

## Pre-Commit Checklist
- [ ] Builds for SITL without errors
- [ ] Builds for CubeOrangePlus without errors (with --enable flags)
- [ ] No std:: symbols in binary
- [ ] Unit tests pass (7/7)
- [ ] Code style check passes
- [ ] SITL automated tests pass (Plane.SensAItion and Plane.SensAItionEAHRS)
