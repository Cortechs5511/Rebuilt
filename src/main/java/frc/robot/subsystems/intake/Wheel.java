package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wheel extends SubsystemBase {
  private static final int WHEEL_MOTOR_ID = 51;
  private static final double MAX_OUTPUT = 0.7;
  private static final double TRIGGER_DEADBAND = 0.05;
  // NEO 550 on the intake axle. Encoder conversion removed because
  // this subsystem currently runs open-loop percent output.
  // For green wheels in intake (CAN ID 51).
  private final SparkMax wheelMotor = new SparkMax(WHEEL_MOTOR_ID, MotorType.kBrushless);

  @SuppressWarnings("deprecation")
  public Wheel() {
    SparkMaxConfig config = new SparkMaxConfig();
    // Use Brake so the intake wheels resist motion when idle.
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(20);
    config.inverted(false);
    // Encoder conversion not configured because we don't read the encoder here.
    wheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Run intake inward at the configured MAX_OUTPUT. */
  public void intakeIn() {
    set(1.0);
  }

  public void setFromTriggers(double leftTrigger, double rightTrigger) {
    double left = MathUtil.applyDeadband(leftTrigger, TRIGGER_DEADBAND);
    double right = MathUtil.applyDeadband(rightTrigger, TRIGGER_DEADBAND);

    // Right trigger pulls in, left trigger deintakes.
    double output = (right - left);
    set(output);
  }

  /** Set wheel speed in normalized [-1.0, 1.0]. Multiplied by MAX_OUTPUT. */
  public void set(double speed) {
    double clamped = MathUtil.clamp(speed, -1.0, 1.0) * MAX_OUTPUT;
    wheelMotor.set(clamped);
  }

  public void stop() {
    wheelMotor.stopMotor();
  }
}
