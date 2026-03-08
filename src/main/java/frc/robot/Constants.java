// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OIConstants {

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final double DEADBAND = 0.15;
    public static final double TRANSLATION_EXPO = 2.0;
    public static final double ROTATION_EXPO = 2.0;
  // Increase this temporarily to test higher translation responsiveness on the robot.
  // 1.0 is nominal; bump up to make the robot respond faster to the same joystick deflection.
  public static final double TELEOP_TRANSLATION_SCALE = 1.4;
    public static final double TELEOP_ROTATION_SCALE = 1.0;
    public static final double TRANSLATION_SLEW_RATE = 6.0;
  // Reduce rotation responsiveness to avoid propulsion from small stick inputs.
  public static final double ROTATION_SLEW_RATE = 4.0;
  }

  public static class SwerveConstants {

    // Motor Controller and Encoder Configuration
    public static final double VOLTAGE_COMPENSATION = 12;
    public static final int DRIVE_CURRENT_LIMIT = 40;
    public static final int ANGLE_CURRENT_LIMIT = 20;
    public static final double RAMP_RATE = 0.25;
    public static final double WHEEL_DIAMETER_IN = 4;
    public static final double WHEEL_CIRCUMFERENCE_IN = WHEEL_DIAMETER_IN*Math.PI;
    public static final double DRIVE_GEAR_RATIO = 8.14;
    public static final double INCHES_PER_METER = 39.3701;
    public static final double TWO_PI = 2.0 * Math.PI;
    // convert native units of rpm to meters per second
    public static final double VELOCITY_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE_IN / DRIVE_GEAR_RATIO / INCHES_PER_METER / 60;
    // convert native motor rotations to wheel meters
    public static final double POSITION_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE_IN / DRIVE_GEAR_RATIO / INCHES_PER_METER;

  // IDs
  // Format for each module: {driveMotorId, turningMotorId, canCoderId}
  // Module order: FL, FR, BL, BR. Values aligned to YAGSL JSON files.
  public static final int[] IDS = {12, 11, 1, 22, 21, 2, 32, 31, 3, 42, 41, 4};
  // Keep module/electrical order aligned with IDS array above: FL, FR, BL, BR.
  public static final boolean[] DRIVE_MOTOR_INVERTED = {false, false, false, false};
  public static final boolean[] TURN_MOTOR_INVERTED = {false, false, false, false};
  // CANcoder mechanical zero offsets were historically stored here, but this
  // project now uses YAGSL deploy JSONs (src/main/deploy/swerve/modules/*.json)
  // as the single source of truth for per-module absolute encoder offsets.
  // The old CANCODER_OFFSETS_RAD array was removed to avoid conflicting
  // calibration sources. If you need to keep offsets in-code, re-add them
  // here or synchronize them from the deploy JSONs at build time.

    // PID Values
    
    
    public static final double[] DRIVE_PID_VALUES = {0.7, 0.0, 0.0};
    public static final double[] TURN_PID_VALUES = {0.73, 0.0, 0.0};

    public static final double ANGLE_MAX_VELOCITY = 7.0;
    public static final double ANGLE_MAX_ACCELERATION = 30.0;
    

    // public static final double PID_RANGE = 0.9;
    // From YAGSL module locations: +/-10.25 in from robot center.
    public static final double MODULE_XY_FROM_CENTER = Units.inchesToMeters(10.25);
    public static final double CHASSIS_LENGTH = MODULE_XY_FROM_CENTER * 2.0;
    public static final double CHASSIS_WIDTH = MODULE_XY_FROM_CENTER * 2.0;
    public static final double[] MODULE_TRANSLATIONS = {
      CHASSIS_LENGTH / 2, CHASSIS_WIDTH / 2, 
      CHASSIS_LENGTH / 2, -CHASSIS_WIDTH / 2,
      -CHASSIS_LENGTH / 2, CHASSIS_WIDTH / 2,
      -CHASSIS_LENGTH / 2, -CHASSIS_WIDTH / 2,
    };

    public static final double DRIVE_BASE_RADIUS = Math.hypot(CHASSIS_LENGTH / 2.0, CHASSIS_WIDTH / 2.0);

    
    // Conservative L1 baseline; tune upward after validation.
    public static final double MAX_TRANSLATIONAL_SPEED = 4.5;
    public static final double MAX_ROTATIONAL_SPEED = MAX_TRANSLATIONAL_SPEED / DRIVE_BASE_RADIUS;


    // Swerve Module Location Constants
    // each module is Math.sqrt(2) * Units.inchesToMeters(23) away from the center

  }

}
