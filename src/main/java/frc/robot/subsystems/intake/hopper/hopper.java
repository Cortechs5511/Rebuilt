package frc.robot.subsystems.intake.hopper;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class hopper extends SubsystemBase {
  private static final int HOPPER_MOTOR_ID = 6;
  private static final double MAX_OUTPUT = 0.7;
  private static final double TRIGGER_DEADBAND = 0.05;

  private final SparkFlex hopperMotor = new SparkFlex(HOPPER_MOTOR_ID, MotorType.kBrushless);

  public hopper() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(40);
    config.inverted(false);
    hopperMotor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void intakeIn() {
    hopperMotor.set(MAX_OUTPUT);
  }

  public void setFromTriggers(double leftTrigger, double rightTrigger) {
    double left = MathUtil.applyDeadband(leftTrigger, TRIGGER_DEADBAND);
    double right = MathUtil.applyDeadband(rightTrigger, TRIGGER_DEADBAND);
    double output = (right - left) * MAX_OUTPUT;
    hopperMotor.set(MathUtil.clamp(output, -1.0, 1.0));
  }

  public void stop() {
    hopperMotor.stopMotor();
  }
}
