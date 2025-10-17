# Kebni SensAItion External AHRS

The Kebni SensAItion range includes three sensor models:
* IMU: Measures acceleration, rotation (gyro rates), magnetic field and air pressure
* AHRS: As above, but also estimates the orientation (roll, pitch and yaw/heading) of the sensor
* INS: As above, but with two built-in satellite navigation receivers to get position (using Fixed Base RTK for higher precision if provided with correction data), UTC time and heading (if using two antennas with a sufficient baseline between them)

For easier integration into various customer applications, each model exists in two hardware versions: a rugged version in an IP67 aluminium housing or a low-profile OEM version that can easily be mounted on an application-specific circuit board. Details and specifications for each model can be found on [Kebni's product homepage](https://www.kebni.com/products/inertial-sensing/sensaition-imu-ins/).

For now, the SensAItion External AHRS driver only supports using the SensAItion sensor to feed measurement values to the ArduPilot EKF3 estimator, so it is suffient to use the IMU model of SensAItion. If using the AHRS version, the estimated orientation quaternion will be available in ArduPilot but not used in the EKF3. There is currently no support for the INS version. 

## Hardware setup
Connect the SensAItion Data UART pins (TX, RX and ground) to an available serial port on the ArduPilot platform, such as GPS or TELEM. Power the sensor as described in the User Manual. Both the rugged and OEM version of the sensor have the required serial port connections available. Please note that the rugged sensor version has output voltage levels +/-5 V, while the OEM sensor version has output voltage levels of 0 and +3.3 V.

The SensAItion sensor can be mounted in any orientation and the output can be rotated to match the local coordinate system if needed - see the configuration section below. As with any IMU, it is important to avoid excessive vibration levels and it is beneficial to use a mechanical damper between the sensor and the platform frame.

## Configuration of the sensor
The SensAItion sensor has a number of configuration registers that control the baudrate, output data format, coordinate system rotation and much more. The configuration registers can be read and written using a text-based protocol via the User UART port as described in the User Manual. But Kebni also provides, on request, the Kebni INSight graphical user interface that helps the user set all configuration registers and can also be used to read and plot sensor data and perform a magnetometer calibration. For complete control of the sensor configuration, it is recommended to use INSight. 

Contact [Kebni](https://www.kebni.com/contact/) for access to the User Manual or Kebni INSight.

To get started with the default configuration, it is sufficient to connect to the User UART port of the sensor with a terminal program.

Start by doing a factory reset of all configuration registers, saving to flash and doing a reboot:
```
$PKEBF*5A
$PKEBS*4F
$PKEBB*5E
```
After getting a reboot message from the sensor, send the following commands to enable Data UART output and set the baudrate to 460800:
```
$PKEBW,7,2*4E
$PKEBW,9,46080*48
```
For an IMU sensor, send the following command that sets an output rate of 1000 Hz and determines the contents of each message (acceleration, gyro rates, temperature, magnetic field and air pressure):
```
$PKEBW,3,o0001s240030020010000130120110100230220210200330320310300430420410400530520510500910900A10A00B10B00C10C00D30D20D10D0x*1B
```
For an AHRS sensor, send the following command that includes all of the above but with reduced output rate of 500 Hz, together with the orientation quaternion in each message:
```
$PKEBW,3,o0002s340030020010000130120110100230220210200330320310300430420410400530520510500910900A10A00B10B00C10C00D30D20D10D04C34C24C14C04D34D24D14D04E34E24E14E04F34F24F14F0x*19
```
Then save the updated register values to flash and reboot to make them active:
```
$PKEBS*4F
$PKEBB*5E
```

## Configuration of ArduPilot
The following parameters must be set:

Parameter | Value | Comment
--------- | ----- | ------------
EAHRS_TYPE | 11 | Use SensAItion as external AHRS
EAHRS_SENSORS | 14 | Use IMU, barometer and compass
EAHRS_RATE | 1000 | Requested rate for AHRS device
EAHRS_OPTIONS | 0 | 0: IMU mode 2: AHRS mode 
AHRS_EKF_TYPE | 3 | Use ArduPilot's EKF3
SERIAL2_PROTOCOL | 36 | External AHRS on TELEM 2 (change to whatever port you are using)
SERIAL2_BAUD | 460 | 460800 baud (change to whatever port you are using)
 

Also remember to build the ArduPilot binary with the flags ```--enable-AHRS_EXT --enable-EXTERNALAHRS_COMPASS --enable-EXTERNALAHRS_BARO```.

## Troubleshooting
Here we add any tips on what can go wrong or how to diagnose the system. 
Maybe explain different error messages that can come from the driver, unless thety are 110% self-explanatory?


