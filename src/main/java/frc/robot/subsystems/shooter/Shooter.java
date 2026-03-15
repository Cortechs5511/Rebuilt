package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simple shooter subsystem with 4 motors:
 * - Two Krakens (implemented here with SparkFlex) with CAN IDs 5 and 6
 * - Two NEOs (Spark Max) with CAN IDs 7 and 9
 *
 * Pressing operator A will spin all shooter wheels (wiring is done in RobotContainer).
 */
public class Shooter extends SubsystemBase {
  // IDs: Krakens use TalonFX controllers with built-in encoders
  private static final int KRAKEN_1_ID = 5;
  private static final int KRAKEN_2_ID = 6;
  private static final int NEO_1_ID = 7;
  private static final int NEO_2_ID = 9; // moved to 9 to avoid conflict with hopper

  // Starting speed used by the operator preset as a high-but-safe initial command.
  // Set slightly below 1.0 so the shooter can spin up reliably without immediately
  // demanding an absolute maximum.
  private static final double DEFAULT_SPEED = 1.00;
  // Per-motor tuning multipliers: bias group outputs to better match wheel
  // linear speeds given the differing wheel diameters. These starting values
  // are recommended for initial on-robot testing; fine-tune empirically.
  private static final double KRAKEN_SPEED_SCALE = 1.60; // increase Kraken contribution
  private static final double NEO_SPEED_SCALE = 1.00; // NEO baseline
  // Counter-rotation direction multipliers for Krakens only. NEO directions
  // are handled via per-controller inversion (configured below) for clarity.
  // Flip Krakens direction (inverted) so positive shooter command spins opposite.
  private static final double KRAKEN_1_DIR = -1.0;
  private static final double KRAKEN_2_DIR = 1.0;
  // Pulley ratios (motor:wheel). Wheel speed = motor speed * (motorPulley / wheelPulley).
  // motor:wheel is the ratio motorPulley/wheelPulley. To compute the motor
  // output required to achieve a wheel speed fraction `s`, we use
  // motorOutput = s * (1.0 / driveRatio) (then normalize if needed so outputs stay in [-1,1]).
  private static final double KRAKEN_DRIVE_RATIO = 24.0 / 30.0;
  private static final double NEO_DRIVE_RATIO = 1.0 / 2.0;

  private final SparkMax neo1 = new SparkMax(NEO_1_ID, MotorType.kBrushless);
  private final SparkMax neo2 = new SparkMax(NEO_2_ID, MotorType.kBrushless);

  // Krakens: use TalonFX (Phoenix 6) controllers which provide encoder reads
  private final TalonFX kraken1 = new TalonFX(KRAKEN_1_ID);
  private final TalonFX kraken2 = new TalonFX(KRAKEN_2_ID);

  public Shooter() {
    // Note: Kraken controllers are not configured here because they are not
    // represented by SparkFlex/SparkMax classes. If your Kraken vendor API
    // requires configuration, perform it here (or add a Kraken-specific
    // config wrapper). For the NEOs we still configure via SparkMaxConfig.
    // Configure NEO controllers. Use per-controller inversion so hardware
    // wiring/tach polarity is clear in config rather than sign hacks in code.
    SparkMaxConfig nConfig1 = new SparkMaxConfig();
    nConfig1.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
    nConfig1.smartCurrentLimit(40);
  nConfig1.inverted(true); // NEO 1 inverted (switched)
    SparkMaxConfig nConfig2 = new SparkMaxConfig();
    nConfig2.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
    nConfig2.smartCurrentLimit(40);
  nConfig2.inverted(false); // NEO 2 not inverted (switched)
    neo1.configure(nConfig1, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kNoPersistParameters);
    neo2.configure(nConfig2, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // Publish shooter diagnostics so operator A preset can be verified on the Dashboard.
    try {
      SmartDashboard.putNumber("Shooter/Neo1Output", neo1.get());
      SmartDashboard.putNumber("Shooter/Neo2Output", neo2.get());
      SmartDashboard.putNumber("Shooter/Kraken1Vel", getKraken1Velocity());
      SmartDashboard.putNumber("Shooter/Kraken2Vel", getKraken2Velocity());
      SmartDashboard.putBoolean("Shooter/AnyActive", Math.abs(neo1.get()) > 0.01 || Math.abs(neo2.get()) > 0.01 || Math.abs(getKraken1Velocity()) > 1.0 || Math.abs(getKraken2Velocity()) > 1.0);
    } catch (Exception e) {
      // Ignore telemetry errors to avoid noisy logs on hardware without telemetry support
    }
  }

  // Encoder-follower control intentionally removed for shooter safety.

  /** Read integrated Kraken encoder position (rotations). */
  public double getKraken1Position() {
    return kraken1.getRotorPosition().getValueAsDouble();
  }

  /** Read integrated Kraken encoder position (rotations). */
  public double getKraken2Position() {
    return kraken2.getRotorPosition().getValueAsDouble();
  }

  /** Read integrated Kraken encoder velocity (rotations per second, RPS). */
  public double getKraken1Velocity() {
    return kraken1.getRotorVelocity().getValueAsDouble();
  }

  /** Read integrated Kraken encoder velocity (rotations per second, RPS). */
  public double getKraken2Velocity() {
    return kraken2.getRotorVelocity().getValueAsDouble();
  }

  /** Spin all shooter wheels at default speed. */
  public void spinAll() {
    spinAll(DEFAULT_SPEED);
  }

  /** Spin all shooter wheels at the given speed ([-1,1]). */
  public void spinAll(double speed) {
    double s = Math.max(-1.0, Math.min(1.0, speed));
    // Convert requested wheel-speed fraction to motor outputs using pulley ratios.
    // The drive ratio is defined as motorPulley/wheelPulley so
    // wheelSpeed = motorSpeed * driveRatio. Therefore motorSpeed = wheelSpeed / driveRatio.
    // We'll compute the raw motor demands and then normalize them if any exceed 1.0.
  double krakenRaw = s / KRAKEN_DRIVE_RATIO; // motor fraction required for Kraken motors
  double neoRaw = s / NEO_DRIVE_RATIO; // motor fraction required for NEO motors

  // Apply per-group tuning to bias Krakens vs NEOs.
  krakenRaw *= KRAKEN_SPEED_SCALE;
  neoRaw *= NEO_SPEED_SCALE;

  // Find max absolute demand to determine if normalization is required.
  double maxDemand = Math.max(Math.abs(krakenRaw), Math.abs(neoRaw));
  double scale = (maxDemand > 1.0) ? (1.0 / maxDemand) : 1.0;

  double krakenOut = krakenRaw * scale;
  double neoOut = neoRaw * scale;

    // Apply outputs. Krakens use explicit direction multipliers; NEO inversion is handled
    // in the controller config above (neo2 configured inverted=true).
    kraken1.setControl(new DutyCycleOut(krakenOut * KRAKEN_1_DIR));
    kraken2.setControl(new DutyCycleOut(krakenOut * KRAKEN_2_DIR));
    neo1.set(neoOut);
    neo2.set(neoOut);
  }

  /** Stop all shooter motors. */
  public void stop() {
    // Stop TalonFX outputs
    kraken1.setControl(new DutyCycleOut(0.0));
    kraken2.setControl(new DutyCycleOut(0.0));
    neo1.stopMotor();
    neo2.stopMotor();
  }
}
