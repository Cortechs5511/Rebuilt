package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
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

  public TeleopSwerve(SwerveSubsystem swerve, CommandXboxController controller) {
    this.swerve = swerve;
    this.controller = controller;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    double forward = -MathUtil.applyDeadband(controller.getLeftY(), OIConstants.DEADBAND);
    double strafe = -MathUtil.applyDeadband(controller.getLeftX(), OIConstants.DEADBAND);
    double rotation = -MathUtil.applyDeadband(controller.getRightX(), OIConstants.DEADBAND);
    forward = Math.copySign(Math.pow(Math.abs(forward), OIConstants.TRANSLATION_EXPO), forward);
    strafe = Math.copySign(Math.pow(Math.abs(strafe), OIConstants.TRANSLATION_EXPO), strafe);
    rotation = Math.copySign(Math.pow(Math.abs(rotation), OIConstants.ROTATION_EXPO), rotation);
      // Field-relative control: forward on stick always moves toward the far end of the field.
    swerve.drive(forward, strafe, rotation, true, false, false);
  }
}

