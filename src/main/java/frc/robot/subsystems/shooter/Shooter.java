package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
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
  private static final int KRAKEN_1_ID = 5;
  private static final int KRAKEN_2_ID = 6;
  private static final int NEO_1_ID = 7;
  private static final int NEO_2_ID = 8;

  private static final double DEFAULT_SPEED = 1.0;

  private final SparkFlex kraken1 = new SparkFlex(KRAKEN_1_ID, MotorType.kBrushless);
  private final SparkFlex kraken2 = new SparkFlex(KRAKEN_2_ID, MotorType.kBrushless);
  private final SparkMax neo1 = new SparkMax(NEO_1_ID, MotorType.kBrushless);
  private final SparkMax neo2 = new SparkMax(NEO_2_ID, MotorType.kBrushless);

  @SuppressWarnings("deprecation")
  public Shooter() {
    // Configure Krakens (SparkFlex)
    SparkFlexConfig kConfig = new SparkFlexConfig();
    kConfig.idleMode(IdleMode.kCoast);
    kConfig.smartCurrentLimit(40);
    kConfig.inverted(false);
    kraken1.configure(kConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    kraken2.configure(kConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Configure NEOs (SparkMax)
    SparkMaxConfig nConfig = new SparkMaxConfig();
    nConfig.idleMode(IdleMode.kCoast);
    nConfig.smartCurrentLimit(40);
    nConfig.inverted(false);
    neo1.configure(nConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    neo2.configure(nConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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
}
