package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simple shooter subsystem with 4 motors:
 * - Two Krakens (implemented here with SparkFlex) with CAN IDs 5 and 6
 * - Two NEOs (Spark Max) with CAN IDs 7 and 8
 *
 * Pressing operator A will spin all shooter wheels (wiring is done in RobotContainer).
 */
public class Shooter extends SubsystemBase {
  // IDs: Krakens use TalonFX controllers with built-in encoders
  private static final int KRAKEN_1_ID = 5;
  private static final int KRAKEN_2_ID = 6;
  private static final int NEO_1_ID = 7;
  private static final int NEO_2_ID = 9; // moved to 9 to avoid conflict with hopper

  private static final double DEFAULT_SPEED = 1.0;

  private final SparkMax neo1 = new SparkMax(NEO_1_ID, MotorType.kBrushless);
  private final SparkMax neo2 = new SparkMax(NEO_2_ID, MotorType.kBrushless);

  // Krakens: use TalonFX (Phoenix 6) controllers which provide encoder reads
  private final TalonFX kraken1 = new TalonFX(KRAKEN_1_ID);
  private final TalonFX kraken2 = new TalonFX(KRAKEN_2_ID);

  // NOTE: encoder follower mode removed for safety — shooters should be
  // commanded at a fixed speed rather than following another motor's
  // measured velocity. Keep a tunable max velocity constant for diagnostics
  // if needed.
  private static final double MAX_ENCODER_VELOCITY = 6000.0; // tune per hardware

  public Shooter() {
    // Note: Kraken controllers are not configured here because they are not
    // represented by SparkFlex/SparkMax classes. If your Kraken vendor API
    // requires configuration, perform it here (or add a Kraken-specific
    // config wrapper). For the NEOs we still configure via SparkMaxConfig.
    SparkMaxConfig nConfig = new SparkMaxConfig();
    nConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
    nConfig.smartCurrentLimit(40);
    nConfig.inverted(false);
    neo1.configure(nConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kNoPersistParameters);
    neo2.configure(nConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // No periodic follower action for the shooter. Use explicit commands
    // (spinAll/stop) to control shooter speed.
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

  /** Read integrated Kraken encoder velocity (RPM or controller units). */
  public double getKraken1Velocity() {
    return kraken1.getRotorVelocity().getValueAsDouble();
  }

  /** Read integrated Kraken encoder velocity (RPM or controller units). */
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
    // Use Phoenix 6 DutyCycleOut control to set percent output on TalonFX.
    kraken1.setControl(new DutyCycleOut(s));
    kraken2.setControl(new DutyCycleOut(s));
    neo1.set(s);
    neo2.set(s);
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
