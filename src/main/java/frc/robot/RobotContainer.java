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
import frc.robot.subsystems.intake.Wheel;

/**
 * Minimal RobotContainer focused on teleop swerve mapping.
 * Left stick = translation (forward/back + strafe), right X = rotation.
 */
public class RobotContainer {
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final Wheel m_intakeWheel = new Wheel();
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    SmartDashboard.putData("Auto chooser", autoChooser);

    // Default teleop swerve mapping: left stick translation, right stick X rotation.
    m_swerveSubsystem.setDefaultCommand(new TeleopSwerve(m_swerveSubsystem, m_driverController));

    // Operator controls intake wheel; driver stays focused on swerve.
    m_intakeWheel.setDefaultCommand(
        Commands.run(
            () ->
                m_intakeWheel.setFromTriggers(
                    m_operatorController.getLeftTriggerAxis(), m_operatorController.getRightTriggerAxis()),
            m_intakeWheel));

    configureBindings();
  }

  private void configureBindings() {
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
