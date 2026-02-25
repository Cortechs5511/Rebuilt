// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.misc.AuxMotorConstants.slowSpeed;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.misc.AuxMotor;
import frc.robot.subsystems.misc.Pivot;
import frc.robot.subsystems.misc.PivotConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final AuxMotor auxMotor;
  private final Pivot pivot;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(1);
  private final CommandXboxController operatorController = new CommandXboxController(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations. Try Pigeon2 first and
        // fall back to NavX if the Pigeon is not present to avoid selecting the wrong
        // gyro implementation (which can cause unexpected LED behavior).
        GyroIO gyro = new GyroIOPigeon2();
        var gyroCheck = new GyroIO.GyroIOInputs();
        try {
          gyro.updateInputs(gyroCheck);
        } catch (Exception e) {
          // If any unexpected error occurs while probing the Pigeon, assume not connected
          gyroCheck.connected = false;
        }
        if (!gyroCheck.connected) {
          gyro = new GyroIONavX();
        }
        // Create modules robustly: try to construct each ModuleIOSpark and probe it.
        // If a module doesn't respond (no SparkMax/CANcoder present), fall back to a
        // noop ModuleIO to avoid crashing during Robot startup when hardware is missing.
        ModuleIO[] modules = new ModuleIO[4];
        for (int i = 0; i < 4; i++) {
          try {
            ModuleIOSpark candidate = new ModuleIOSpark(i);
            var probe = new ModuleIO.ModuleIOInputs();
            try {
              candidate.updateInputs(probe);
            } catch (Exception ignored) {
              // If probing throws, we'll treat the module as not present
            }
            if (!probe.driveConnected && !probe.turnConnected) {
              modules[i] = new ModuleIO() {};
            } else {
              modules[i] = candidate;
            }
          } catch (Exception e) {
            // Construction failed; use noop stub and continue
            e.printStackTrace();
            modules[i] = new ModuleIO() {};
          }
        }

        drive = new Drive(gyro, modules[0], modules[1], modules[2], modules[3]);
        auxMotor = new AuxMotor(true);
        pivot = new Pivot(true);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        auxMotor = new AuxMotor(false);
        pivot = new Pivot(false);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        auxMotor = new AuxMotor(false);
        pivot = new Pivot(false);
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command: translation on left stick and continuous rotation from
    // the right stick X axis (deadbanding and squaring handled in DriveCommands).
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Reduced-speed field-relative drive while holding A; maintain current heading
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY() * 0.55,
                () -> -controller.getLeftX() * 0.55,
                // Without aimbot: keep the robot's current heading
                () -> drive.getRotation()));
    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Run auxiliary motor with right trigger at slow speed
    auxMotor.setDefaultCommand(
        Commands.run(
            () -> auxMotor.setPercent(-controller.getRightTriggerAxis() * slowSpeed), auxMotor));

    // Pivot control on second controller left joystick (Y axis)
    pivot.setDefaultCommand(
        Commands.run(
            () ->
                pivot.setPercent(
                    MathUtil.applyDeadband(-pivotController.getLeftY(), PivotConstants.deadband)
                        * PivotConstants.maxOutput),
            pivot));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
