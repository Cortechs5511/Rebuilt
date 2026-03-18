package frc.robot.subsystems.intake;

<<<<<<< HEAD
// Intake pivot subsystem (lowercase class name to match file `intake.java`).
// Implementation includes soft limits, PID hold, and safe encoder seeding.

=======
>>>>>>> f0a761e460a9184d8456fa0681426b74c1b7342b
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
<<<<<<< HEAD
import edu.wpi.first.wpilibj.DriverStation;
=======
>>>>>>> f0a761e460a9184d8456fa0681426b74c1b7342b
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
  // Minimum (most-retracted / rearward) safe pivot rotation. Tune on robot.
<<<<<<< HEAD
  private static final double MIN_PIVOT_ROT = -0.50;
  private static final double MAX_PIVOT_ROT = 1.50;
=======
  // Negative means rotated rearward from the zero reference.
  private static final double MIN_PIVOT_ROT = -0.50; // ~ -180deg (adjust after testing)
  private static final double MAX_PIVOT_ROT = 1.50; // safe forward limit (adjust after testing)
>>>>>>> f0a761e460a9184d8456fa0681426b74c1b7342b
  private static final double STOWED_POSITION_ROT = MIN_PIVOT_ROT;
  private static final double INTAKE_POSITION_ROT = 18.0 / PIVOT_GEAR_RATIO;

  private final SparkMax pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
  private final PIDController pivotController = new PIDController(PIVOT_KP, PIVOT_KI, PIVOT_KD);

  private boolean positionControlEnabled = true;
  private double targetPositionRot = STOWED_POSITION_ROT;

<<<<<<< HEAD
  @SuppressWarnings("deprecation") // REVLib configure() API pending upstream fix
  public intake() {
    SparkMaxConfig config = new SparkMaxConfig();
    // Start in Brake mode — position control will hold the arm on enable.
=======
  @SuppressWarnings("deprecation")
  public intake() {
    SparkMaxConfig config = new SparkMaxConfig();
>>>>>>> f0a761e460a9184d8456fa0681426b74c1b7342b
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(25);
    config.inverted(false);
    // Expose arm rotations in software instead of raw motor rotations.
    config.encoder.positionConversionFactor(1.0 / PIVOT_GEAR_RATIO);
    config.encoder.velocityConversionFactor(1.0 / PIVOT_GEAR_RATIO);
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
<<<<<<< HEAD

    // Seed the encoder with the actual reading clamped to the safe travel range.
    // This guards against out-of-range values at boot due to encoder or wiring issues.
    double initial = MathUtil.clamp(pivotEncoder.getPosition(), MIN_PIVOT_ROT, MAX_PIVOT_ROT);
    pivotEncoder.setPosition(initial);

    pivotController.setTolerance(PIVOT_TOLERANCE_ROT);
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
=======
    // Clamp initial encoder position to a safe range in case the encoder value
    // or robot wiring yields an out-of-range value at boot.
    double initial = MathUtil.clamp(STOWED_POSITION_ROT, MIN_PIVOT_ROT, MAX_PIVOT_ROT);
    pivotEncoder.setPosition(initial);
    pivotController.setTolerance(PIVOT_TOLERANCE_ROT);
  }

  public void setPivotFromJoystick(double rightY) {
    double output = MathUtil.applyDeadband(rightY, PIVOT_DEADBAND) * MAX_MANUAL_OUTPUT;
    if (Math.abs(output) > 0.0) {
      positionControlEnabled = false;
      pivotMotor.set(clampPivotOutput(output));
      return;
    }

    if (!positionControlEnabled) {
      // When leaving manual control, capture current position as the new
      // target (clamped) so the PID can hold the arm and prevent falling.
      setTargetPositionRot(getPivotPositionRot());
    }
  }

>>>>>>> f0a761e460a9184d8456fa0681426b74c1b7342b
  public void moveToIntakePosition() {
    setTargetPositionRot(INTAKE_POSITION_ROT);
  }

<<<<<<< HEAD
  /** Command the pivot to the stowed (retracted) position. */
=======
>>>>>>> f0a761e460a9184d8456fa0681426b74c1b7342b
  public void moveToStowedPosition() {
    setTargetPositionRot(STOWED_POSITION_ROT);
  }

<<<<<<< HEAD
  /**
   * Set the PID target position, clamped to the safe travel range.
   * Enables position control mode.
   */
  public void setTargetPositionRot(double targetRot) {
=======
  public void setTargetPositionRot(double targetRot) {
    // keep targets inside the safe travel range
>>>>>>> f0a761e460a9184d8456fa0681426b74c1b7342b
    targetPositionRot = MathUtil.clamp(targetRot, MIN_PIVOT_ROT, MAX_PIVOT_ROT);
    positionControlEnabled = true;
  }

<<<<<<< HEAD
  /** Returns the current pivot position in arm rotations. */
=======
>>>>>>> f0a761e460a9184d8456fa0681426b74c1b7342b
  public double getPivotPositionRot() {
    return pivotEncoder.getPosition();
  }

<<<<<<< HEAD
  /** Returns true when the pivot is within tolerance of the target position. */
  public boolean atTargetPosition() {
    return pivotController.atSetpoint();
=======
  public boolean atTargetPosition() {
    return Math.abs(targetPositionRot - getPivotPositionRot()) <= PIVOT_TOLERANCE_ROT;
>>>>>>> f0a761e460a9184d8456fa0681426b74c1b7342b
  }

  @Override
  public void periodic() {
    if (positionControlEnabled) {
      double output = pivotController.calculate(getPivotPositionRot(), targetPositionRot);
      pivotMotor.set(clampPivotOutput(output));
    }
<<<<<<< HEAD
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
=======

  }

  public void stop() {
    positionControlEnabled = false;
    pivotMotor.stopMotor();
  }

  private double clampPivotOutput(double requestedOutput) {
    double maxDown = POSITIVE_OUTPUT_MOVES_DOWN ? MAX_PIVOT_OUTPUT_DOWN : MAX_PIVOT_OUTPUT_UP;
    double maxUp = POSITIVE_OUTPUT_MOVES_DOWN ? MAX_PIVOT_OUTPUT_UP : MAX_PIVOT_OUTPUT_DOWN;
    return MathUtil.clamp(requestedOutput, -maxUp, maxDown);
  }
}
>>>>>>> f0a761e460a9184d8456fa0681426b74c1b7342b
