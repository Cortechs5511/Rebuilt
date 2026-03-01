package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wheel extends SubsystemBase {
  private static final int WHEEL_MOTOR_ID = 6;
  private static final double MAX_OUTPUT = 0.7;
  private static final double TRIGGER_DEADBAND = 0.05;

  // Hopper pulley motor (NEO Vortex on Spark Flex).
  private final SparkFlex wheelMotor = new SparkFlex(WHEEL_MOTOR_ID, MotorType.kBrushless);

  public Wheel() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(40);
    config.inverted(false);
    wheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void intakeIn() {
    wheelMotor.set(MAX_OUTPUT);
  }

  public void setFromTriggers(double leftTrigger, double rightTrigger) {
    double left = MathUtil.applyDeadband(leftTrigger, TRIGGER_DEADBAND);
    double right = MathUtil.applyDeadband(rightTrigger, TRIGGER_DEADBAND);

    // Right trigger pulls in, left trigger deintakes.
    double output = (right - left) * MAX_OUTPUT;
    wheelMotor.set(MathUtil.clamp(output, -1.0, 1.0));
  }

  public void stop() {
    wheelMotor.stopMotor();
  }
}
