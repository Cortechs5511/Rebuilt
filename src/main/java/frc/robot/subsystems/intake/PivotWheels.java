package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotWheels extends SubsystemBase {
  private static final int PIVOT_WHEELS_MOTOR_ID = 52;
  private static final double INTAKE_OUTPUT = 0.9;

  private final SparkMax pivotWheelsMotor =
      new SparkMax(PIVOT_WHEELS_MOTOR_ID, MotorType.kBrushless);

  @SuppressWarnings("deprecation")
  public PivotWheels() {
    SparkMaxConfig config = new SparkMaxConfig();
    // Use Brake so the small pivot wheels provide passive resistance when stopped.
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(20);
    config.inverted(false);
    // This motor is a regular NEO (direct-driven pivot wheels). Do not apply
    // gearbox conversion here — encoder reports motor rotations directly.
    pivotWheelsMotor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void intakeIn() {
    // Delegate to set() so clamping and any future logic stays in one place.
    set(INTAKE_OUTPUT);
  }

  /** Set pivot wheels speed directly ([-1,1]). Right trigger -> positive. */
  public void set(double speed) {
    pivotWheelsMotor.set(MathUtil.clamp(speed, -1.0, 1.0));
  }

  public void stop() {
    pivotWheelsMotor.stopMotor();
  }
}
