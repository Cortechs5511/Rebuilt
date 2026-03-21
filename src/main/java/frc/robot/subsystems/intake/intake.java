package frc.robot.subsystems.intake;

// Intake pivot subsystem (lowercase class name to match file `intake.java`).
// Implementation includes soft limits, PID hold, and safe encoder seeding.

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake extends SubsystemBase {
  private static final int PIVOT_MOTOR_ID = 13;
  private static final double PIVOT_GEAR_RATIO = 45.0; // motor rotations : arm rotations
  private static final double PIVOT_DEADBAND = 0.10;
  private static final double MAX_MANUAL_OUTPUT = 0.30;
  private static final double MAX_PIVOT_OUTPUT_DOWN = 0.18;
  private static final double MAX_PIVOT_OUTPUT_UP = 0.30;
  // If direction is opposite on robot, flip this to false.
  private static final boolean POSITIVE_OUTPUT_MOVES_DOWN = true;
  private static final double PIVOT_KP = 0.08;
  private static final double PIVOT_KI = 0.0;
  private static final double PIVOT_KD = 0.0;
  private static final double PIVOT_TOLERANCE_ROT = 0.75 / PIVOT_GEAR_RATIO;
  // Small gravity feed-forward to help the PID hold the arm under load.
  // Tune this on the robot; start small (0.03 - 0.08) and increase as needed.
  private static final double PIVOT_GRAVITY_FF = 0.06;
  // Minimum (most-retracted / rearward) safe pivot rotation. Tune on robot.
  private static final double MIN_PIVOT_ROT = -0.50;
  private static final double MAX_PIVOT_ROT = 1.50;
  // Keep the stowed target slightly above the hard soft-limit so the PID
  // doesn't fight the limit clamp directly. Teams should tune this value
  // on the bench; 0.05 arm-rotations (~18deg at a 45:1 gear) is a reasonable
  // safety margin to avoid oscillation at the limit.
  private static final double STOWED_POSITION_ROT = MIN_PIVOT_ROT + 0.05;
  private static final double INTAKE_POSITION_ROT = 18.0 / PIVOT_GEAR_RATIO;

  private final SparkMax pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
  private final PIDController pivotController = new PIDController(PIVOT_KP, PIVOT_KI, PIVOT_KD);

  private boolean positionControlEnabled = true;
  private double targetPositionRot;
  // Preferences key to enable/disable automatic latch-on-enable behavior
  private static final String PREF_LATCH_ON_ENABLE = "Intake/LatchOnEnable";
  // Preferences key for persisted stowed preset (arm rotations)
  private static final String PREF_STOWED_KEY = "Intake/StowedPositionRot";
  // Preferences key: when true, automatically persist any latched stowed
  // value to PREF_STOWED_KEY. Default true for convenience; teams can
  // disable if they prefer manual save behavior.
  private static final String PREF_AUTO_PERSIST_LATCH = "Intake/AutoPersistOnLatch";
  // Preferences key for persisted elevated preset (arm rotations)
  private static final String PREF_ELEVATED_KEY = "Intake/ElevatedPositionRot";
  private final boolean latchOnEnable;

  // Track previous disabled state to detect transitions
  private boolean wasDisabled = true;

  @SuppressWarnings("deprecation") // REVLib configure() API pending upstream fix
  public intake() {
    SparkMaxConfig config = new SparkMaxConfig();
  // Start in Coast mode. Change to Coast so the motor is not actively
  // resisting motion when disabled; teams may prefer Brake if they want
  // passive holding while disabled.
  config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(25);
    config.inverted(false);
    // Expose arm rotations in software instead of raw motor rotations.
    config.encoder.positionConversionFactor(1.0 / PIVOT_GEAR_RATIO);
    config.encoder.velocityConversionFactor(1.0 / PIVOT_GEAR_RATIO);
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  // Seed the encoder with the actual reading clamped to the safe travel range.
  // This guards against out-of-range values at boot due to encoder or wiring issues.
  double initial = MathUtil.clamp(pivotEncoder.getPosition(), MIN_PIVOT_ROT, MAX_PIVOT_ROT);
  pivotEncoder.setPosition(initial);

    pivotController.setTolerance(PIVOT_TOLERANCE_ROT);

    // Read preference to decide whether to latch current position when the
    // robot transitions from disabled -> enabled. Default true for safety
    // (keeps the arm where the driver left it at enable).
    latchOnEnable = Preferences.getBoolean(PREF_LATCH_ON_ENABLE, true);
    wasDisabled = DriverStation.isDisabled();

    // Load persisted stowed preset if present. This seeds the relative encoder
    // so the arm will treat the saved rotation as the true position on boot.
    if (Preferences.containsKey(PREF_STOWED_KEY)) {
      double saved = Preferences.getDouble(PREF_STOWED_KEY, STOWED_POSITION_ROT);
      saved = MathUtil.clamp(saved, MIN_PIVOT_ROT, MAX_PIVOT_ROT);
      pivotEncoder.setPosition(saved);
      targetPositionRot = saved;
      positionControlEnabled = true;
      DriverStation.reportWarning("Intake: loaded saved stowed preset = " + saved, false);
    } else {
      // No persisted stowed preset: initialize the PID target to the current
      // encoder reading (safer than forcing the STOWED constant which could
      // command motion if the encoder isn't aligned to the physical arm).
      targetPositionRot = MathUtil.clamp(pivotEncoder.getPosition(), MIN_PIVOT_ROT, MAX_PIVOT_ROT);
    }
    // Load persisted elevated preset presence only; do not change current
    // target until explicitly requested (we don't auto-move to elevated).
    if (Preferences.containsKey(PREF_ELEVATED_KEY)) {
      double savedE = Preferences.getDouble(PREF_ELEVATED_KEY, INTAKE_POSITION_ROT);
      savedE = MathUtil.clamp(savedE, MIN_PIVOT_ROT, MAX_PIVOT_ROT);
      DriverStation.reportWarning("Intake: found saved elevated preset = " + savedE, false);
    }
  }

  /**
   * Drive the pivot from a joystick axis.
   *
   * <p>While the joystick is deflected the arm runs in open-loop. When the
   * stick returns to center the current position is latched as the new PID
   * target so the arm holds in place rather than falling.
   */
  public void setPivotFromJoystick(double rightY) {
    double output = MathUtil.applyDeadband(rightY, PIVOT_DEADBAND) * MAX_MANUAL_OUTPUT;

    if (Math.abs(output) > 0.0) {
      // Entering / staying in manual control: disable PID and drive open-loop.
      positionControlEnabled = false;
      pivotMotor.set(clampPivotOutput(output));
    } else if (!positionControlEnabled) {
      // Joystick just returned to centre: latch the current position so the
      // PID holds the arm. setTargetPositionRot() also re-enables position control.
      setTargetPositionRot(getPivotPositionRot());
    }
    // If positionControlEnabled is already true and output == 0, periodic()
    // handles motor output — no action needed here.
  }

  /** Command the pivot to the intake (deployed) position. */
  public void moveToIntakePosition() {
    setTargetPositionRot(INTAKE_POSITION_ROT);
  }

  /** Command the pivot to the stowed (retracted) position. */
  public void moveToStowedPosition() {
    setTargetPositionRot(STOWED_POSITION_ROT);
  }

  /**
   * Calibrate the encoder by declaring the current physical location to be
   * the configured STOWED position. This should be run only while the robot
   * is disabled and the arm is physically placed in the stowed pose.
   */
  public void calibrateCurrentPositionAsStowed() {
    if (!DriverStation.isDisabled()) {
      DriverStation.reportWarning("Intake calibration must be run while robot is disabled. Aborting.", false);
      return;
    }

    pivotEncoder.setPosition(STOWED_POSITION_ROT);
    targetPositionRot = STOWED_POSITION_ROT;
    positionControlEnabled = true;
    DriverStation.reportWarning("Intake pivot calibrated: current physical position set to STOWED.", false);
  }

  /**
   * Calibrate the encoder to the current sensor reading (useful when you
   * physically align the arm to a mark and want to accept the encoder value
   * as the true position). Must be run while disabled.
   */
  public void calibrateToCurrentSensorReading() {
    if (!DriverStation.isDisabled()) {
      DriverStation.reportWarning("Intake calibration must be run while robot is disabled. Aborting.", false);
      return;
    }

    double cur = MathUtil.clamp(pivotEncoder.getPosition(), MIN_PIVOT_ROT, MAX_PIVOT_ROT);
    pivotEncoder.setPosition(cur);
    targetPositionRot = cur;
    positionControlEnabled = true;
    DriverStation.reportWarning("Intake pivot calibrated to current sensor reading.", false);
  }

  /**
   * Persist the current calibrated elevated position so it survives reboots.
   * Must be run while robot is disabled and when the arm is physically at
   * the desired elevated pose.
   */
  public void saveElevatedPresetFromCurrent() {
    if (!DriverStation.isDisabled()) {
      DriverStation.reportWarning("Intake save preset must be run while robot is disabled. Aborting.", false);
      return;
    }

    double cur = MathUtil.clamp(pivotEncoder.getPosition(), MIN_PIVOT_ROT, MAX_PIVOT_ROT);
    Preferences.setDouble(PREF_ELEVATED_KEY, cur);
    DriverStation.reportWarning("Saved Intake/ElevatedPositionRot = " + cur, false);
  }

  /**
   * Move to the persisted elevated preset if one exists. Returns true when a
   * saved preset was applied, false otherwise.
   */
  public boolean moveToPersistedElevatedIfPresent() {
    if (!Preferences.containsKey(PREF_ELEVATED_KEY)) return false;
    double saved = Preferences.getDouble(PREF_ELEVATED_KEY, INTAKE_POSITION_ROT);
    saved = MathUtil.clamp(saved, MIN_PIVOT_ROT, MAX_PIVOT_ROT);
    setTargetPositionRot(saved);
    return true;
  }

  /**
   * Persist the current calibrated stowed position so it survives reboots.
   * Must be run while robot is disabled and when the arm is physically at
   * the desired stowed pose.
   */
  public void saveStowedPresetFromCurrent() {
    if (!DriverStation.isDisabled()) {
      DriverStation.reportWarning("Intake save preset must be run while robot is disabled. Aborting.", false);
      return;
    }

    double cur = MathUtil.clamp(pivotEncoder.getPosition(), MIN_PIVOT_ROT, MAX_PIVOT_ROT);
    Preferences.setDouble(PREF_STOWED_KEY, cur);
    DriverStation.reportWarning("Saved Intake/StowedPositionRot = " + cur, false);
  }

  /**
   * Persist a provided numeric value (in arm rotations) as the stowed preset.
   * This allows entering the numeric encoder value shown on Shuffleboard and
   * storing it as the fixed stowed position. Must be run while disabled.
   */
  public void saveStowedPresetFromValue(double valueRot) {
    if (!DriverStation.isDisabled()) {
      DriverStation.reportWarning("Intake save preset must be run while robot is disabled. Aborting.", false);
      return;
    }

    double cur = MathUtil.clamp(valueRot, MIN_PIVOT_ROT, MAX_PIVOT_ROT);
    Preferences.setDouble(PREF_STOWED_KEY, cur);
    DriverStation.reportWarning("Saved Intake/StowedPositionRot (from value) = " + cur, false);
  }

  /**
   * Move to the persisted stowed preset if one exists. Returns true when a
   * saved preset was applied, false otherwise.
   */
  public boolean moveToPersistedStowedIfPresent() {
    if (!Preferences.containsKey(PREF_STOWED_KEY)) return false;
    double saved = Preferences.getDouble(PREF_STOWED_KEY, STOWED_POSITION_ROT);
    saved = MathUtil.clamp(saved, MIN_PIVOT_ROT, MAX_PIVOT_ROT);
    setTargetPositionRot(saved);
    return true;
  }

  /**
   * Set the PID target position, clamped to the safe travel range.
   * Enables position control mode.
   */
  public void setTargetPositionRot(double targetRot) {
    targetPositionRot = MathUtil.clamp(targetRot, MIN_PIVOT_ROT, MAX_PIVOT_ROT);
    positionControlEnabled = true;
  }

  /** Returns the current pivot position in arm rotations. */
  public double getPivotPositionRot() {
    return pivotEncoder.getPosition();
  }

  /** Returns true when the pivot is within tolerance of the target position. */
  public boolean atTargetPosition() {
    return pivotController.atSetpoint();
  }

  @Override
  public void periodic() {
    // Detect disabled -> enabled transition and latch current position so
    // the arm will hold wherever it physically is at the moment of enable.
    boolean disabled = DriverStation.isDisabled();

    // New: allow operators to press a Shuffleboard/SmartDashboard boolean
    // "Intake/LatchNow" while DISABLED to immediately latch the current
    // encoder position and use it as the PID target. This is useful on the
    // bench to set the desired pose before enabling the robot. The widget
    // should be a toggle/button that writes true; we reset it to false after
    // handling so a single press is enough. Optionally auto-persist the latched
    // value based on PREF_AUTO_PERSIST_LATCH (default true).
    try {
      boolean latchNow = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getBoolean("Intake/LatchNow", false);
      if (disabled && latchNow) {
        double cur = MathUtil.clamp(pivotEncoder.getPosition(), MIN_PIVOT_ROT, MAX_PIVOT_ROT);
        // Align the relative encoder to the physical pose and set the PID target.
        pivotEncoder.setPosition(cur);
        targetPositionRot = cur;
        positionControlEnabled = true; // will actually hold once enabled
        // Optionally persist the latched stowed preset so it survives reboot.
        boolean autoPersist = Preferences.getBoolean(PREF_AUTO_PERSIST_LATCH, true);
        if (autoPersist) {
          Preferences.setDouble(PREF_STOWED_KEY, cur);
          DriverStation.reportWarning("Intake: auto-saved stowed preset = " + cur, false);
        }
        // Publish latched value for visibility and clear the widget so the operator doesn't need to toggle it back.
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Intake/LatchedValue", cur);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Intake/LatchNow", false);
        DriverStation.reportWarning("Intake: latched position from Shuffleboard while disabled: " + cur, false);
      }
    } catch (Exception e) {
      // Defensive: do not allow dashboard read failures to break periodic.
    }

    if (latchOnEnable && wasDisabled && !disabled) {
      // Just enabled: latch current encoder reading as the target and optionally persist.
      double cur = MathUtil.clamp(pivotEncoder.getPosition(), MIN_PIVOT_ROT, MAX_PIVOT_ROT);
      setTargetPositionRot(cur);
      boolean autoPersist = Preferences.getBoolean(PREF_AUTO_PERSIST_LATCH, true);
      if (autoPersist) {
        Preferences.setDouble(PREF_STOWED_KEY, cur);
        DriverStation.reportWarning("Intake: auto-saved stowed preset on enable = " + cur, false);
      }
      // Publish latched value for visibility
      edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Intake/LatchedValue", cur);
      DriverStation.reportWarning("Intake: latched current position on enable: " + cur, false);
    }
    wasDisabled = disabled;

    if (positionControlEnabled) {
      double output = pivotController.calculate(getPivotPositionRot(), targetPositionRot);
      // Simple gravity compensation: assume pivotEncoder returns arm rotations
      // and convert to radians. Feed-forward is kG * sin(theta).
      double angleRad = getPivotPositionRot() * 2.0 * Math.PI;
      double gravityFF = PIVOT_GRAVITY_FF * Math.sin(angleRad);
      pivotMotor.set(clampPivotOutput(output + gravityFF));
    }

    // Publish current encoder reading so operators can copy the numeric
    // value from Shuffleboard/SmartDashboard and use it to persist presets.
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Intake/EncoderRot", pivotEncoder.getPosition());
  }

  /** Cuts motor output and disables position control. */
  public void stop() {
    positionControlEnabled = false;
    pivotMotor.stopMotor();
    DriverStation.reportWarning("Intake pivot stopped.", false);
  }

  /**
   * Clamps the requested motor output to the configured directional limits,
   * and enforces soft travel limits so the arm cannot be driven past
   * MIN_PIVOT_ROT or MAX_PIVOT_ROT even under open-loop or PID overshoot.
   */
  private double clampPivotOutput(double requestedOutput) {
    double pos = getPivotPositionRot();

    // Soft limit: block motion that would drive further past a boundary.
    // "Down" is the positive direction when POSITIVE_OUTPUT_MOVES_DOWN is true.
    boolean movingDown = (requestedOutput > 0) == POSITIVE_OUTPUT_MOVES_DOWN;
    if (pos <= MIN_PIVOT_ROT && movingDown) return 0.0;
    if (pos >= MAX_PIVOT_ROT && !movingDown) return 0.0;

    double maxDown = POSITIVE_OUTPUT_MOVES_DOWN ? MAX_PIVOT_OUTPUT_DOWN : MAX_PIVOT_OUTPUT_UP;
    double maxUp   = POSITIVE_OUTPUT_MOVES_DOWN ? MAX_PIVOT_OUTPUT_UP   : MAX_PIVOT_OUTPUT_DOWN;
    return MathUtil.clamp(requestedOutput, -maxUp, maxDown);
  }
}