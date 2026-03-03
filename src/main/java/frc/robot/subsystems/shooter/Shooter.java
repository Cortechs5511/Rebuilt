package frc.robot.subsystems.shooter;

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
  private static final int Kraken_1_EncoderValue = 5;
  private static final int Kraken_2_EncoderValue = 6;
  private static final int NEO_1_ID = 7;
  private static final int NEO_2_ID = 8;

  private static final double DEFAULT_SPEED = 1.0;

  private final SparkMax neo1 = new SparkMax(NEO_1_ID, MotorType.kBrushless);
  private final SparkMax neo2 = new SparkMax(NEO_2_ID, MotorType.kBrushless);

  // Kraken CAN IDs (controllers that provide integrated encoders)
  private static final int KRAKEN_1_ID = 5;
  private static final int KRAKEN_2_ID = 6;

  // Krakens: these controllers expose integrated encoders but are NOT
  // represented by SparkMax/SparkFlex in this codebase. We create a small
  // platform-agnostic abstraction here that holds the encoder ID and a
  // placeholder for commanding the motor. Replace the TODOs below with
  // your real hardware API (Kraken vendor library) when available.
  private final KrakenController kraken1 = new KrakenController(KRAKEN_1_ID);
  private final KrakenController kraken2 = new KrakenController(KRAKEN_2_ID);

  // Encoder IDs (logical): Krakens expose integrated encoders; we document
  // the controller IDs that host those encoders. Use the kraken#getEncoder()
  // API to read positions/velocity — do not create a separate encoder
  // device with its own CAN ID.
  private static final int ENCODER_1_ID = KRAKEN_1_ID; // logical mapping
  private static final int ENCODER_2_ID = KRAKEN_2_ID; // logical mapping

  // Simple follower mode: when enabled the shooter will read the encoder
  // velocities and command the motors to follow those values (scaled).
  private boolean encoderFollowerEnabled = false;
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
    // If encoder follower mode is enabled, read encoder velocities from
    // the Kraken controllers and command the motors to follow that
    // velocity (scaled to [-1,1]). This lets an external encoder
    // measurement drive the shooter outputs.
    if (encoderFollowerEnabled) {
      double v1 = kraken1.getEncoder().getVelocity();
      double v2 = kraken2.getEncoder().getVelocity();
      double s1 = Math.max(-1.0, Math.min(1.0, v1 / MAX_ENCODER_VELOCITY));
      double s2 = Math.max(-1.0, Math.min(1.0, v2 / MAX_ENCODER_VELOCITY));
      // Command Kraken outputs. KrakenController#set(...) is a hardware
      // placeholder — implement it to call your real Kraken motor API.
      kraken1.set(s1);
      kraken2.set(s2);
      // Mirror to NEOs so all shooter wheels spin together
      neo1.set((s1 + s2) / 2.0);
      neo2.set((s1 + s2) / 2.0);
    }
  }

  /** Enable/disable encoder follower mode. */
  public void setEncoderFollowerEnabled(boolean enabled) {
    encoderFollowerEnabled = enabled;
  }

  /** Read integrated Kraken encoder position (rotations). */
  public double getKraken1Position() {
    return kraken1.getEncoder().getPosition();
  }

  /** Read integrated Kraken encoder position (rotations). */
  public double getKraken2Position() {
    return kraken2.getEncoder().getPosition();
  }

  /** Read integrated Kraken encoder velocity (RPM or controller units). */
  public double getKraken1Velocity() {
    return kraken1.getEncoder().getVelocity();
  }

  /** Read integrated Kraken encoder velocity (RPM or controller units). */
  public double getKraken2Velocity() {
    return kraken2.getEncoder().getVelocity();
  }

  /** Spin all shooter wheels at default speed. */
  public void spinAll() {
    spinAll(DEFAULT_SPEED);
  }

  /** Spin all shooter wheels at the given speed ([-1,1]). */
  public void spinAll(double speed) {
    double s = Math.max(-1.0, Math.min(1.0, speed));
    kraken1.set(s);
    kraken2.set(s);
    neo1.set(s);
    neo2.set(s);
  }

  /** Stop all shooter motors. */
  public void stop() {
    kraken1.stopMotor();
    kraken2.stopMotor();
    neo1.stopMotor();
    neo2.stopMotor();
  }

  // --- Kraken abstraction (replace with vendor API) ---
  private static class KrakenEncoder {
    private final int id;

    KrakenEncoder(int id) {
      this.id = id;
    }

    /** Position in rotations. Implement real read from Kraken hardware here. */
    public double getPosition() {
      // TODO: read from CAN/device by id
      return 0.0;
    }

    /** Velocity in controller units (e.g., RPM). Implement real read here. */
    public double getVelocity() {
      // TODO: read from CAN/device by id
      return 0.0;
    }
  }

  private static class KrakenController {
    private final int id;
    private final KrakenEncoder encoder;
    private double lastOutput = 0.0;

    KrakenController(int id) {
      this.id = id;
      this.encoder = new KrakenEncoder(id);
    }

    public KrakenEncoder getEncoder() {
      return encoder;
    }

    /** Command motor output in [-1,1]. Implement to call Kraken motor API. */
    public void set(double output) {
      lastOutput = Math.max(-1.0, Math.min(1.0, output));
      // TODO: send output to Kraken motor controller via vendor API
    }

    public void stopMotor() {
      set(0.0);
    }
  }
}
