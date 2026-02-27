package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private SparkMax driveMotor;
    private SparkMax turnMotor;
    private CoreCANcoder absoluteEncoder;
    private final double absoluteOffsetRad;

    private RelativeEncoder driveEncoder;


    public SwerveModule(int driveMotorPort, int turningMotorPort, int absoluteEncoderPort, boolean driveInverted, boolean turnInverted, double angleOffsetRad) {
        driveMotor = createMotorController(driveMotorPort, driveInverted);
        turnMotor = createMotorController(turningMotorPort, turnInverted);
        absoluteEncoder = new CoreCANcoder(absoluteEncoderPort);
        absoluteOffsetRad = angleOffsetRad;

        driveEncoder = createEncoder(driveMotor);
    }

    private SparkMax createMotorController(int port, boolean isInverted) {
        SparkMax controller = new SparkMax(port, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();

        config.voltageCompensation(SwerveConstants.VOLTAGE_COMPENSATION);
        config.idleMode(IdleMode.kBrake);
        config.openLoopRampRate(SwerveConstants.RAMP_RATE);
        config.closedLoopRampRate(SwerveConstants.RAMP_RATE); 

        config.smartCurrentLimit(SwerveConstants.CURRENT_LIMIT);

        config.inverted(isInverted);

        config.encoder.velocityConversionFactor(SwerveConstants.VELOCITY_CONVERSION_FACTOR);
        config.encoder.positionConversionFactor(SwerveConstants.POSITION_CONVERSION_FACTOR);
        // for some reason causes robot to shake:
        //     controller.burnFlash(); 
        controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        return controller;
    }

    private RelativeEncoder createEncoder(SparkMax controller) {
        RelativeEncoder encoder = controller.getEncoder();
        // convert from native unit of rpm to m/s
        //encoder.setVelocityConversionFactor(SwerveConstants.VELOCITY_CONVERSION_FACTOR);

        return encoder;
    }

    public void setTargetState(SwerveModuleState targetState, PIDController drivePID, ProfiledPIDController turnPID) {
        // get angle + optimize angle 
        Rotation2d currentAngle = getAngle();
        // Optimize the desired state relative to the current angle. Use the static optimize helper
        // so we get the optimized state back.
        SwerveModuleState optimizedState = SwerveModuleState.optimize(targetState, currentAngle);

        // calculate the motor output + set the motor states
        // Add a normalized feedforward term so requested wheel speed maps directly to motor output.
        double driveFeedforward = optimizedState.speedMetersPerSecond / SwerveConstants.MAX_TRANSLATIONAL_SPEED;
        double driveFeedback = drivePID.calculate(getVelocity(), optimizedState.speedMetersPerSecond);
        double driveOutput = driveFeedforward + driveFeedback;
        double turnOutput = turnPID.calculate(getAngle().getRadians(), optimizedState.angle.getRadians());

        // Suppress small angle-hunting around setpoint to reduce module twitch.
        double angleErrorRad = MathUtil.angleModulus(optimizedState.angle.getRadians() - currentAngle.getRadians());
        if (Math.abs(angleErrorRad) < Math.toRadians(1.5)) {
            turnOutput = 0.0;
        }

        driveOutput = MathUtil.clamp(driveOutput, -1.0, 1.0);
        turnOutput = MathUtil.clamp(turnOutput, -1.0, 1.0);
        
        turnMotor.set(turnOutput);
        driveMotor.set(driveOutput);
    }

    // var delta = angle.minus(currentAngle);
    // if (Math.abs(delta.getDegrees()) > 90.0) {
    //   speedMetersPerSecond *= -1;
    //   angle = angle.rotateBy(Rotation2d.kPi);
    // }

    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getAbsoluteEncoderPos() {
        // Phoenix6 absolute position is in rotations [0,1). Convert to radians and apply module offset.
        double absoluteRotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        return (absoluteRotations * SwerveConstants.TWO_PI) - absoluteOffsetRad;
    }

    public double getAbsoluteEncoderDegrees() {
        return Math.toDegrees(getAbsoluteEncoderPos());
    }

    public Rotation2d getAngle() {
        // Wrap to [-pi, pi] to match continuous-input PID configuration.
        return Rotation2d.fromRadians(MathUtil.angleModulus(getAbsoluteEncoderPos()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }
}
