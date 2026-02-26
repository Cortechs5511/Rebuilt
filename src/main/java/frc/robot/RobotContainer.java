// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/**
 * Minimal RobotContainer focused on teleop swerve mapping.
 * Left stick = translation (forward/back + strafe), right X = rotation.
 */
public class RobotContainer {
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    SmartDashboard.putData("Auto chooser", autoChooser);

    // Single default command to avoid conflicting teleop behavior.
    m_swerveSubsystem.setDefaultCommand(new TeleopSwerve(m_swerveSubsystem, m_driverController));

    configureBindings();
  }

  private void configureBindings() {
    // Hold A: translation only, no rotation.
    m_driverController.a().whileTrue(
        Commands.run(
            () ->
                m_swerveSubsystem.drive(
                    -m_driverController.getLeftY(),
                    -m_driverController.getLeftX(),
                    0.0,
                    true,
                    false,
                    false),
            m_swerveSubsystem));

    // Press X: stop with wheel X-lock.
    m_driverController.x().onTrue(Commands.runOnce(m_swerveSubsystem::stopWithX, m_swerveSubsystem));

    // Press B: reset gyro heading to 0 degrees.
    m_driverController.b().onTrue(Commands.runOnce(() -> m_swerveSubsystem.resetGyro(0.0), m_swerveSubsystem));
  }

  /** Returns selected autonomous command or null. */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
