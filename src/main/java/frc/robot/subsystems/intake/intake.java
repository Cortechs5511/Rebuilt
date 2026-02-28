package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake extends SubsystemBase {
  private static final int PIVOT_MOTOR_ID = 5;
  private static final double PIVOT_DEADBAND = 0.10;
  private static final double MAX_PIVOT_OUTPUT = 0.45;
  private static final double PIVOT_KP = 0.08;
  private static final double PIVOT_KI = 0.0;
  private static final double PIVOT_KD = 0.0;
  private static final double PIVOT_TOLERANCE_ROT = 0.75;
  private static final double STOWED_POSITION_ROT = 0.0;
  private static final double INTAKE_POSITION_ROT = 18.0;

  private final SparkMax pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
  private final PIDController pivotController = new PIDController(PIVOT_KP, PIVOT_KI, PIVOT_KD);

  private boolean positionControlEnabled = true;
  private double targetPositionRot = STOWED_POSITION_ROT;

  public intake() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(25);
    config.inverted(false);
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    pivotEncoder.setPosition(STOWED_POSITION_ROT);
    pivotController.setTolerance(PIVOT_TOLERANCE_ROT);
  }

  public void setPivotFromJoystick(double rightY) {
    double output = MathUtil.applyDeadband(rightY, PIVOT_DEADBAND) * MAX_PIVOT_OUTPUT;
    if (Math.abs(output) > 0.0) {
      positionControlEnabled = false;
      pivotMotor.set(MathUtil.clamp(output, -1.0, 1.0));
      return;
    }

    if (!positionControlEnabled) {
      setTargetPositionRot(getPivotPositionRot());
    }
  }

  public void moveToIntakePosition() {
    setTargetPositionRot(INTAKE_POSITION_ROT);
  }

  public void moveToStowedPosition() {
    setTargetPositionRot(STOWED_POSITION_ROT);
  }

  public void setTargetPositionRot(double targetRot) {
    targetPositionRot = targetRot;
    positionControlEnabled = true;
  }

  public double getPivotPositionRot() {
    return pivotEncoder.getPosition();
  }

  public boolean atTargetPosition() {
    return Math.abs(targetPositionRot - getPivotPositionRot()) <= PIVOT_TOLERANCE_ROT;
  }

  @Override
  public void periodic() {
    if (positionControlEnabled) {
      double output = pivotController.calculate(getPivotPositionRot(), targetPositionRot);
      pivotMotor.set(MathUtil.clamp(output, -MAX_PIVOT_OUTPUT, MAX_PIVOT_OUTPUT));
    }

    SmartDashboard.putNumber("Intake/Pivot Position Rot", getPivotPositionRot());
    SmartDashboard.putNumber("Intake/Pivot Target Rot", targetPositionRot);
    SmartDashboard.putBoolean("Intake/Pivot At Target", atTargetPosition());
  }

  public void stop() {
    positionControlEnabled = false;
    pivotMotor.stopMotor();
  }
}
