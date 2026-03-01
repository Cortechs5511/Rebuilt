// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.intake.AprilTag;
import frc.robot.subsystems.intake.PivotWheels;
import frc.robot.subsystems.auto.BlueLeftAuto;
import frc.robot.subsystems.intake.hopper.hopper;
import frc.robot.subsystems.intake.intake;


/**
 * Minimal RobotContainer focused on teleop swerve mapping.
 * Left stick = translation (forward/back + strafe), right X = rotation.
 */
public class RobotContainer {
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final AprilTag m_aprilTag = new AprilTag();
  private final hopper m_hopper = new hopper();
  private final PivotWheels m_pivotWheels = new PivotWheels();
  private final intake m_intakePivot = new intake();
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    SmartDashboard.putData("Auto chooser", autoChooser);

    // Default teleop swerve mapping: left stick translation, right stick X rotation.
    m_swerveSubsystem.setDefaultCommand(new TeleopSwerve(m_swerveSubsystem, m_driverController));

    // Operator controls intake pivot with right Y axis.
    m_intakePivot.setDefaultCommand(
        Commands.run(
            () -> m_intakePivot.setPivotFromJoystick(m_operatorController.getRightY()), m_intakePivot));

    // Operator controls hopper with triggers.
    m_hopper.setDefaultCommand(
        Commands.run(
            () ->
                m_hopper.setFromTriggers(
                    m_operatorController.getLeftTriggerAxis(), m_operatorController.getRightTriggerAxis()),
            m_hopper));

    autoChooser.setDefaultOption("BlueLeftAuto", BlueLeftAuto.build(m_swerveSubsystem, m_aprilTag));

    configureBindings();
  }

  private void configureBindings() {
    // Hold X on driver controller for intake preset:
    // lower intake bar + run pivot wheels + run blue intake wheels.
    m_driverController
        .x()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  m_intakePivot.moveToIntakePosition();
                  m_pivotWheels.intakeIn();
                  m_hopper.intakeIn();
                },
                () -> {
                  m_pivotWheels.stop();
                  m_hopper.stop();
                },
                m_intakePivot,
                m_pivotWheels,
                m_hopper));

    // Press B: reset gyro heading to 0 degrees.
    m_driverController.b().onTrue(Commands.runOnce(() -> m_swerveSubsystem.resetGyro(0.0), m_swerveSubsystem));

    // Hold A on operator controller for explicit intake action.
    m_operatorController
        .a()
        .whileTrue(Commands.startEnd(m_hopper::intakeIn, m_hopper::stop, m_hopper));

    // Press Y to scan visible AprilTag IDs and run the configured action.
    // Current configured action: if ID 23 is found, rotate to 45 degrees.
    m_driverController
        .y()
        .onTrue(createScanAndRunTagCommand());

  }

  private Command createScanAndRunTagCommand() {
    final int targetTagId = 23;
    final double targetHeadingDeg = 45.0;
    final double headingKp = 2.4;
    final double maxOmegaRadPerSec = 2.0;
    final double headingToleranceDeg = 2.0;

    return Commands.sequence(
        Commands.runOnce(() -> m_aprilTag.updateOriginFromAlliance(edu.wpi.first.wpilibj.DriverStation.getAlliance())),
        Commands.waitUntil(() -> m_aprilTag.getBestVisibleTarget()
            .map(target -> target.getFiducialId() == targetTagId)
            .orElse(false)).withTimeout(1.5),
        Commands.run(
            () -> {
              double currentHeadingRad = m_swerveSubsystem.getPose().getRotation().getRadians();
              double targetHeadingRad = Math.toRadians(targetHeadingDeg);
              double errorRad = MathUtil.angleModulus(targetHeadingRad - currentHeadingRad);
              double omega = MathUtil.clamp(errorRad * headingKp, -maxOmegaRadPerSec, maxOmegaRadPerSec);
              m_swerveSubsystem.driveFieldRelative(new ChassisSpeeds(0.0, 0.0, omega));
            },
            m_swerveSubsystem)
            .until(() -> {
              double currentHeadingRad = m_swerveSubsystem.getPose().getRotation().getRadians();
              double targetHeadingRad = Math.toRadians(targetHeadingDeg);
              return Math.abs(MathUtil.angleModulus(targetHeadingRad - currentHeadingRad))
                  <= Math.toRadians(headingToleranceDeg);
            })
            .withTimeout(1.5),
        Commands.runOnce(m_swerveSubsystem::stop, m_swerveSubsystem));
  }

  /** Returns selected autonomous command or null. */
  public Command getAutonomousCommand() {
    Command selected = autoChooser.getSelected();
    if (selected != null) {
      return selected;
    }
    DriverStation.reportWarning("Auto chooser returned null, falling back to BlueLeftAuto.", false);
    return BlueLeftAuto.build(m_swerveSubsystem, m_aprilTag);
  }
}
