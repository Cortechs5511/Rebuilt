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
    public static final double TELEOP_TRANSLATION_SCALE = 1.0;
    public static final double TELEOP_ROTATION_SCALE = 1.0;
    public static final double TRANSLATION_SLEW_RATE = 6.0;
    public static final double ROTATION_SLEW_RATE = 8.0;
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
  // CANcoder mechanical zero offsets in radians (module order: FL, FR, BL, BR).
  // Values derived from current on-robot readings so current module pose is treated as zero.
  public static final double[] CANCODER_OFFSETS_RAD = {1.47262, 1.56773, -1.56926, -2.66299};

    // PID Values
    //
    // -- How to calibrate swerve PID values --------------------------------------
    // DRIVE PID  (controls wheel speed in m/s)
    //   1. Set I and D to 0. Start with P around 0.01.
    //   2. Slowly raise P until the wheel tracks commanded speed without oscillating.
    //      Symptom of too-high P: wheel surges or oscillates at the setpoint.
    //   3. If steady-state speed error remains at a constant setpoint, add a small I
    //      (e.g. 0.001) and raise until the error is corrected within ~0.5 s.
    //   4. If overshoot or oscillation appears after adding I, add D (e.g. 0.001) to
    //      dampen. Keep D small - too large will amplify sensor noise.
    //   5. The normalised feedforward term (speed / MAX_TRANSLATIONAL_SPEED) already
    //      handles the bulk of the motor output, so P values are intentionally small.
    //
    // TURN PID  (ProfiledPIDController, controls wheel angle in radians)
    //   1. Set I and D to 0. Set ANGLE_MAX_VELOCITY and ANGLE_MAX_ACCELERATION to
    //      conservative values (e.g. 4 rad/s, 8 rad/s^2).
    //   2. Raise P until the wheel snaps to its target angle quickly without hunting.
    //      Symptom of too-high P: module oscillates / buzzes around the target angle.
    //   3. Add D (e.g. 0.005) to damp oscillation while keeping response crisp.
    //      I is rarely needed for the turn loop; leave it at 0.
    //   4. Once P and D are stable, increase ANGLE_MAX_VELOCITY / ANGLE_MAX_ACCELERATION
    //      to allow faster turns; re-check for oscillation after each increase.
    //   5. Use the TURN_DEADBAND_DEG constant to silence small residual twitching
    //      without affecting real tracking; keep it <= 3 deg or module accuracy suffers.
    // ----------------------------------------------------------------------------
    public static final double[] DRIVE_PID_VALUES = {0.015, 0.0, 0.0};
    public static final double[] TURN_PID_VALUES = {0.35, 0.0, 0.005};

    public static final double ANGLE_MAX_VELOCITY = 4.0;
    public static final double ANGLE_MAX_ACCELERATION = 12.0;
    

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

    
    // Deadband constants
    //
    // There are three separate deadband layers in the swerve stack - each serves a
    // distinct purpose and they do NOT compound one another's effect:
    //
    //  1. OIConstants.DEADBAND (0.15, normalised)
    //       Applied to raw controller axis values in TeleopSwerve to mask stick drift.
    //       After applyDeadband the output is linearly remapped so full stick = 1.0.
    //
    //  2. CHASSIS_LINEAR_DEADBAND_MPS / CHASSIS_ANGULAR_DEADBAND_RAD_PER_SEC
    //       Applied to the robot-relative ChassisSpeeds request.  When ALL three
    //       velocity components are below their threshold the drive is stopped and
    //       module angles are frozen, preventing idle angle-hunting.
    //
    //  3. SwerveModule DRIVE_REQUEST_DEADBAND_MPS / TURN_DEADBAND_DEG
    //       Per-module guards: suppress sub-threshold drive output (chatter) and
    //       tiny residual turn corrections (buzz near setpoint) at the motor level.
    //
    // Tuning advice: raise CHASSIS_LINEAR/ANGULAR thresholds if modules still hunt
    // at idle; lower them if the robot feels sluggish to start from rest.
    public static final double CHASSIS_LINEAR_DEADBAND_MPS = 0.02;
    public static final double CHASSIS_ANGULAR_DEADBAND_RAD_PER_SEC = 0.02;

    // Conservative L1 baseline; tune upward after validation.
    public static final double MAX_TRANSLATIONAL_SPEED = 4.5;
    public static final double MAX_ROTATIONAL_SPEED = MAX_TRANSLATIONAL_SPEED / DRIVE_BASE_RADIUS;


    // Swerve Module Location Constants
    // each module is Math.sqrt(2) * Units.inchesToMeters(23) away from the center

  }

}
