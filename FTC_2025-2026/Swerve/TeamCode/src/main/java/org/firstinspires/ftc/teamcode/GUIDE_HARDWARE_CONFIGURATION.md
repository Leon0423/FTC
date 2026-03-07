# Swerve Drive Hardware Configuration

## Required Hardware Configuration

To fix the "driveMotor8" error, you need to configure your robot's hardware with the following names:

### Drive Motors (DcMotorEx)
- **FL** - Front Left Drive Motor
- **FR** - Front Right Drive Motor  
- **BL** - Back Left Drive Motor
- **BR** - Back Right Drive Motor

### Turning Servos (CRServo)
- **FLTurn** - Front Left Turning Servo
- **FRTurn** - Front Right Turning Servo
- **BLTurn** - Back Left Turning Servo
- **BRTurn** - Back Right Turning Servo

### Absolute Encoders (AnalogInput)
- **FLEncoder** - Front Left Absolute Encoder
- **FREncoder** - Front Right Absolute Encoder
- **BLEncoder** - Back Left Absolute Encoder  
- **BREncoder** - Back Right Absolute Encoder

### IMU
- **imu** - IMU sensor for robot heading

## Steps to Configure:

1. Open the FTC Driver Station app
2. Go to "Configure Robot" 
3. Edit your current configuration or create a new one
4. Add the hardware devices with the exact names listed above
5. Assign each device to the correct port on your Control Hub/Expansion Hub
6. Save the configuration

## Alternative: Modify Hardware Names

If you prefer to use different hardware names, you can edit the Constants.java file and change:

```java
// Drive Motors
public static final String kFrontLeftDriveMotorName = "FL";
public static final String kBackLeftDriveMotorName = "BL"; 
public static final String kFrontRightDriveMotorName = "FR";
public static final String kBackRightDriveMotorName = "BR";

// Turning Servos
public static final String kFrontLeftTurningMotorName = "FLTurn";
public static final String kBackLeftTurningMotorName = "BLTurn";
public static final String kFrontRightTurningMotorName = "FRTurn"; 
public static final String kBackRightTurningMotorName = "BRTurn";

// Absolute Encoders
public static final String kFrontLeftAbsoluteEncoderName = "FLEncoder";
public static final String kFrontRightAbsoluteEncoderName = "FREncoder";
public static final String kBackLeftAbsoluteEncoderName = "BLEncoder";
public static final String kBackRightAbsoluteEncoderName = "BREncoder";
```

Change these to match your actual hardware configuration names.
