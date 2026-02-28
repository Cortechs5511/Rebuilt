package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wheel extends SubsystemBase {
  private static final int WHEEL_MOTOR_ID = 51;
  private static final double MAX_OUTPUT = 0.7;
  private static final double TRIGGER_DEADBAND = 0.05;

  // Positive output is defined as clockwise for this intake wheel.
  private final SparkMax wheelMotor = new SparkMax(WHEEL_MOTOR_ID, MotorType.kBrushless);

  public Wheel() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(40);
    config.inverted(false);
    wheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void intakeIn() {
    wheelMotor.set(0.9);
  }

  public void setFromTriggers(double leftTrigger, double rightTrigger) {
    double left = MathUtil.applyDeadband(leftTrigger, TRIGGER_DEADBAND);
    double right = MathUtil.applyDeadband(rightTrigger, TRIGGER_DEADBAND);

    // Left trigger = clockwise, right trigger = counterclockwise.
    double output = (left - right) * MAX_OUTPUT;
    wheelMotor.set(MathUtil.clamp(output, -1.0, 1.0));
  }

  public void stop() {
    wheelMotor.stopMotor();
  }
}
