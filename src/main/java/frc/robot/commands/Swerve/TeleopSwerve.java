package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/**
 * Simple teleop command for swerve using CommandXboxController.
 * Left stick -> translation (forward/back + strafe)
 * Right stick X -> rotation
 */
public class TeleopSwerve extends Command {
  private final SwerveSubsystem swerve;
  private final CommandXboxController controller;
  private final SlewRateLimiter forwardLimiter =
      new SlewRateLimiter(OIConstants.TRANSLATION_SLEW_RATE);
  private final SlewRateLimiter strafeLimiter =
      new SlewRateLimiter(OIConstants.TRANSLATION_SLEW_RATE);
  private final SlewRateLimiter rotationLimiter =
      new SlewRateLimiter(OIConstants.ROTATION_SLEW_RATE);

  public TeleopSwerve(SwerveSubsystem swerve, CommandXboxController controller) {
    this.swerve = swerve;
    this.controller = controller;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    forwardLimiter.reset(0.0);
    strafeLimiter.reset(0.0);
    rotationLimiter.reset(0.0);
  }

  @Override
  public void execute() {
  // Raw inputs (apply deadband to avoid joystick noise)
  double rawForward = -MathUtil.applyDeadband(controller.getLeftY(), OIConstants.DEADBAND);
  double rawStrafe = -MathUtil.applyDeadband(controller.getLeftX(), OIConstants.DEADBAND);
  double rawRotation = -MathUtil.applyDeadband(controller.getRightX(), OIConstants.DEADBAND);

  // Apply exponential response (gives finer control near zero) and scaling
  double expForward = Math.copySign(Math.pow(Math.abs(rawForward), OIConstants.TRANSLATION_EXPO), rawForward)
    * OIConstants.TELEOP_TRANSLATION_SCALE;
  double expStrafe = Math.copySign(Math.pow(Math.abs(rawStrafe), OIConstants.TRANSLATION_EXPO), rawStrafe)
    * OIConstants.TELEOP_TRANSLATION_SCALE;
  // Send raw rotation input to the subsystem; rotation scaling and
  // translation-dependent reduction are handled centrally in SwerveSubsystem.
  double expRotation = rawRotation;

  // Slew-rate limit the processed commands to prevent sudden jumps/propulsion
  double forward = forwardLimiter.calculate(expForward);
  double strafe = strafeLimiter.calculate(expStrafe);
  double rotation = rotationLimiter.calculate(expRotation);
  SmartDashboard.putNumber("Swerve/Diag/DriverRawForward", rawForward);
  SmartDashboard.putNumber("Swerve/Diag/DriverRawStrafe", rawStrafe);
  SmartDashboard.putNumber("Swerve/Diag/DriverRawRotate", rawRotation);
  SmartDashboard.putNumber("Swerve/Diag/DriverForwardCmd", forward);
  SmartDashboard.putNumber("Swerve/Diag/DriverStrafeCmd", strafe);
  SmartDashboard.putNumber("Swerve/Diag/DriverRotateCmd", rotation);
    SmartDashboard.putBoolean(
        "Swerve/Diag/DriverInputActive",
        Math.abs(forward) > 0.05 || Math.abs(strafe) > 0.05 || Math.abs(rotation) > 0.05);

  swerve.drive(forward, strafe, rotation, true, false, false);
  }
}
