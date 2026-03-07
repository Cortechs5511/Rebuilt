package frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private Rotation2d gyroOffset = new Rotation2d();
    private ChassisSpeeds lastRequestedRobotRelativeSpeeds = new ChassisSpeeds();

    public SwerveSubsystem() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(SwerveConstants.MAX_TRANSLATIONAL_SPEED);
        } catch (IOException e) {
            throw new RuntimeException("Failed to load YAGSL swerve JSON config from " + swerveJsonDirectory, e);
        }

        gyroOffset = new Rotation2d();
        RobotConfig config;
        try {
        config = RobotConfig.fromGUISettings();

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                // PathPlanner provides robot-relative speeds in SI units.
                // Send them directly to avoid applying speed scaling a second time.
                (speeds, feedforwards) -> driveRobotRelative(speeds),
                // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(SwerveConstants.DRIVE_PID_VALUES[0], SwerveConstants.DRIVE_PID_VALUES[1], SwerveConstants.DRIVE_PID_VALUES[2]), // Translation PID constants
                        new PIDConstants(SwerveConstants.TURN_PID_VALUES[0], SwerveConstants.TURN_PID_VALUES[1], SwerveConstants.TURN_PID_VALUES[2]) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements
        );
        } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
        }
    }


        public void resetGyro(double degrees) { 
            swerveDrive.setGyro(new Rotation3d(0.0, 0.0, Math.toRadians(degrees)));
            resetPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(degrees)));
            gyroOffset = new Rotation2d();
        }

    @Override 
    public void periodic () {
        logStates(); 
    }


    public void drive(double y, double x, double theta, boolean fieldRelative, boolean alignLimelight, boolean resetGyro) {
        ChassisSpeeds newDesiredSpeeds; 
        
        if (alignLimelight) { 
            newDesiredSpeeds = new ChassisSpeeds(y, x, theta);
        } else { 
            newDesiredSpeeds = new ChassisSpeeds(
            SwerveConstants.MAX_TRANSLATIONAL_SPEED * y, 
            SwerveConstants.MAX_TRANSLATIONAL_SPEED * x,
            SwerveConstants.MAX_ROTATIONAL_SPEED * theta
        );
        }

        // reset gyro button
        if (resetGyro) {
            gyroOffset = swerveDrive.getOdometryHeading();
        }

        // implementing field logic
        if (fieldRelative) {
            driveFieldRelative(newDesiredSpeeds);
        }
        else {
            driveRobotRelative(newDesiredSpeeds);
        }

        // driveRobotRelative(newDesiredSpeeds);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public ChassisSpeeds getSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    public SwerveModulePosition[] getPositions() {
        return swerveDrive.getModulePositions();
    } 

    public void resetPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        Rotation2d drivingAngle = swerveDrive.getOdometryHeading().minus(gyroOffset);
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, drivingAngle));
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        lastRequestedRobotRelativeSpeeds = robotRelativeSpeeds;

        final double linearDeadbandMps = SwerveConstants.CHASSIS_LINEAR_DEADBAND_MPS;
        final double angularDeadbandRadPerSec = SwerveConstants.CHASSIS_ANGULAR_DEADBAND_RAD_PER_SEC;

        // When no chassis motion is requested, hold module angles to prevent idle hunting.
        if (Math.abs(robotRelativeSpeeds.vxMetersPerSecond) < linearDeadbandMps
                && Math.abs(robotRelativeSpeeds.vyMetersPerSecond) < linearDeadbandMps
                && Math.abs(robotRelativeSpeeds.omegaRadiansPerSecond) < angularDeadbandRadPerSec) {
            stop();
            return;
        }

        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        swerveDrive.setChassisSpeeds(targetSpeeds);
    }

    public void setStates(SwerveModuleState[] targetStates) {
        swerveDrive.setModuleStates(targetStates, false);
    }


    public SwerveModuleState[] getStates() {
        return swerveDrive.getStates();
    }

    public void logStates() {
        SwerveModuleState[] currentStates = swerveDrive.getStates();
        double maxAbsDriveCommand = 0.0;
        for (SwerveModuleState state : currentStates) {
            maxAbsDriveCommand = Math.max(maxAbsDriveCommand, Math.abs(state.speedMetersPerSecond));
        }

        boolean requestedMotion =
                Math.abs(lastRequestedRobotRelativeSpeeds.vxMetersPerSecond) > 0.10
                || Math.abs(lastRequestedRobotRelativeSpeeds.vyMetersPerSecond) > 0.10
                || Math.abs(lastRequestedRobotRelativeSpeeds.omegaRadiansPerSecond) > 0.10;
        boolean commandReachingModules = maxAbsDriveCommand > 0.10;

        SmartDashboard.putBoolean("Swerve/Diag/RequestedMotion", requestedMotion);
        SmartDashboard.putBoolean("Swerve/Diag/CommandReachingModules", commandReachingModules);
        SmartDashboard.putNumber("Swerve/Diag/MaxDriveCmdAbs", maxAbsDriveCommand);
    }
    
    /**
     * Stop driving (zero wheel speeds) but keep current wheel angles.
     */
    public void stop() {
        swerveDrive.setChassisSpeeds(new ChassisSpeeds());
    }

    /**
     * Stop and set modules to an X pattern (wheels at +/-45deg) to resist being pushed.
     */
    public void stopWithX() {
        swerveDrive.lockPose();
    }
    
}
