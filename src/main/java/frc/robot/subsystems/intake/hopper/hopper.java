package frc.robot.subsystems.intake.hopper;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class hopper extends SubsystemBase {
  private static final int HOPPER_MOTOR_ID = 17;
  private static final int HOPPER_AUX_NEO_ID = 25;
  private static final double MAX_OUTPUT = 0.7;
  private static final double TRIGGER_DEADBAND = 0.05;
  // Ratios are motor:hopper-shaft.
  private static final double PRIMARY_GEAR_RATIO = 1.0;
  private static final double AUX_GEAR_RATIO = 3.0;
  // Shared sign flip so positive feed command matches physical "feed in" direction.
  private static final double HOPPER_FEED_DIRECTION = -1.0;

  // Small per-motor boosts to slightly increase output for specific CAN IDs
  // (ID 17 = primary SparkFlex, ID 25 = auxiliary NEO). Values >1.0 increase
  // commanded output; keep modest to avoid saturating or stressing hardware.
  private static final double HOPPER_PRIMARY_SPEED_BOOST = 1.6; // ~+20%
  // Make ID 25 (aux) faster than the primary conveyor (ID 17).
  private static final double HOPPER_AUX_SPEED_BOOST = 2.0; // ~+40%

  private final SparkFlex hopperMotor = new SparkFlex(HOPPER_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax hopperAuxNeo = new SparkMax(HOPPER_AUX_NEO_ID, MotorType.kBrushless);

  @SuppressWarnings("deprecation")
  public hopper() {
  SparkFlexConfig config = new SparkFlexConfig();
  config.idleMode(IdleMode.kCoast);
  config.smartCurrentLimit(40);
  // Invert the primary SparkFlex so its physical rotation matches the
  // intended feed direction. If the motor spins the wrong way on robot,
  // flip this flag.
  config.inverted(true);
  // Neo vortex is 1:1 gear ratio on the hopper so expose wheel rotations directly
  // (motor rotations = wheel rotations). This sets encoder conversions to 1:1.
  config.encoder.positionConversionFactor(1.0);
  config.encoder.velocityConversionFactor(1.0);
  hopperMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  SparkMaxConfig auxConfig = new SparkMaxConfig();
  auxConfig.idleMode(IdleMode.kCoast);
  auxConfig.smartCurrentLimit(40);
  // Invert auxiliary NEO so its physical rotation matches intended intake direction
  // (ID 25). This flips output direction without changing higher-level command math.
  // Flipped to 'false' so ID 25 spins in the opposite direction compared to
  // the previous configuration.
  auxConfig.inverted(false);
  // NEO on id 25 has 3:1 motor:shaft reduction.
  auxConfig.encoder.positionConversionFactor(1.0 / AUX_GEAR_RATIO);
  auxConfig.encoder.velocityConversionFactor(1.0 / AUX_GEAR_RATIO);
  hopperAuxNeo.configure(auxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void intakeIn() {
    setBothFromCommand(MAX_OUTPUT);
  }

  /** Run the hopper in reverse (de-intake) at MAX_OUTPUT magnitude. */
  public void intakeOut() {
    setBothFromCommand(-MAX_OUTPUT);
  }

  public void setFromTriggers(double leftTrigger, double rightTrigger) {
    double left = MathUtil.applyDeadband(leftTrigger, TRIGGER_DEADBAND);
    double right = MathUtil.applyDeadband(rightTrigger, TRIGGER_DEADBAND);
    double output = (right - left) * MAX_OUTPUT;
    setBothFromCommand(output);
  }

  public void stop() {
    hopperMotor.stopMotor();
    hopperAuxNeo.stopMotor();
  }

  // Convert the shared hopper command into per-motor output with gear-ratio compensation.
  private void setBothFromCommand(double command) {
    double clamped = MathUtil.clamp(command, -1.0, 1.0) * HOPPER_FEED_DIRECTION;
    double primaryScale = PRIMARY_GEAR_RATIO;
    double auxScale = AUX_GEAR_RATIO;
    double norm = Math.max(primaryScale, auxScale);

  // Compute per-motor outputs and apply small boost factors for IDs 17/25.
  double primaryOut = MathUtil.clamp(clamped * (primaryScale / norm) * HOPPER_PRIMARY_SPEED_BOOST, -1.0, 1.0);
  double auxOut = MathUtil.clamp(clamped * (auxScale / norm) * HOPPER_AUX_SPEED_BOOST, -1.0, 1.0);

    // Apply outputs after the shared feed-direction flip.
    hopperMotor.set(primaryOut);
    hopperAuxNeo.set(auxOut);

    // Telemetry to aid debugging on the driver station / Shuffleboard
    SmartDashboard.putNumber("Hopper/Commanded", command);
    SmartDashboard.putNumber("Hopper/PrimaryOut", primaryOut);
    SmartDashboard.putNumber("Hopper/AuxOut", auxOut);
  }
}
