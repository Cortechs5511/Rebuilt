package frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    // TEMPORARY: flip to true to disable YAGSL + swerve initialization at deploy
    private static final boolean TEMP_DISABLE_SWERVE = true;

    // When TEMP_DISABLE_SWERVE is true, swerveDrive will remain null and all
    // methods that would touch hardware / YAGSL will no-op or return safe
    // defaults. This is a temporary mitigation for deploy-time looping.
    private SwerveDrive swerveDrive;
    private final boolean swerveEnabled;
    private Rotation2d gyroOffset = new Rotation2d();
    // Hardware gyro wrapper used for raw IMU telemetry and zeroing
    private final Gyro hardwareGyro = new Gyro();
    private ChassisSpeeds lastRequestedRobotRelativeSpeeds = new ChassisSpeeds();
    // Heading-hold PID (simple P controller used for teleop heading hold)
    private final PIDController headingPid = new PIDController(2.4, 0.0, 0.0);
    private boolean holdHeading = false;
    private double holdHeadingRad = 0.0;

    public SwerveSubsystem() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
        // Respect the temp disable flag so teams can deploy without YAGSL
        // initializing while debugging.
        swerveEnabled = !TEMP_DISABLE_SWERVE;

        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        if (!swerveEnabled) {
            // Skip heavy YAGSL initialization when disabled
            SmartDashboard.putBoolean("Swerve/DisabledAtDeploy", true);
            // Ensure underlying IMU yaw is zeroed for telemetry
            hardwareGyro.setYaw(0.0);
            swerveDrive = null;
            return;
        }
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(SwerveConstants.MAX_TRANSLATIONAL_SPEED);
        } catch (IOException e) {
            throw new RuntimeException("Failed to load YAGSL swerve JSON config from " + swerveJsonDirectory, e);
        }

        // Apply a configurable startup gyro reset so the robot's "front" can be
        // aligned without code changes. Set the value via Preferences key
        // "Swerve/initialHeadingDegrees" (default 0.0). This will call resetGyro
        // and also reset odometry to the specified heading on boot.
        gyroOffset = new Rotation2d();
        double initialHeadingDeg = Preferences.getDouble("Swerve/initialHeadingDegrees", 0.0);
        if (Math.abs(initialHeadingDeg) > 0.000001) {
            // Apply initial heading (will set swerveDrive gyro + reset odometry)
            resetGyro(initialHeadingDeg);
        } else {
            // Ensure underlying IMU yaw is zeroed on boot. This explicitly
            // writes the Pigeon yaw register so transient device state from
            // prior runs doesn't produce an unexpected heading.
            hardwareGyro.setYaw(0.0);
        }
        RobotConfig config;
        try {
        config = RobotConfig.fromGUISettings();

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> drive(
                        speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond,
                        false,
                        false,
                        false),
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
            if (!swerveEnabled) {
            return;
            }

            swerveDrive.setGyro(new Rotation3d(0.0, 0.0, Math.toRadians(degrees)));
            resetPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(degrees)));
            gyroOffset = new Rotation2d();
        }

        /**
         * Enable heading hold: the subsystem will PID-control robot rotation to maintain the
         * requested absolute heading (radians, field-relative) while still accepting translation
         * commands. Call {@link #clearHoldHeading()} to disable.
         */
        public void holdHeading(double headingRad) {
            holdHeading = true;
            holdHeadingRad = headingRad;
            headingPid.reset();
        }

        /** Disable heading hold. */
        public void clearHoldHeading() {
            holdHeading = false;
        }

    @Override 
    public void periodic () {
        if (!swerveEnabled) {
            // Avoid interacting with YAGSL/swerve drive when disabled.
            SmartDashboard.putBoolean("Swerve/DisabledPeriodic", true);
            return;
        }

        logStates(); 
        // If heading hold is active, compute rotation command from PID and apply
        if (holdHeading) {
            double currentHeading = swerveDrive.getOdometryHeading().getRadians();
            double error = MathUtil.angleModulus(holdHeadingRad - currentHeading);
            double omega = headingPid.calculate(error, 0.0);
            // Clamp to maximum rotational speed
            omega = MathUtil.clamp(omega, -SwerveConstants.MAX_ROTATIONAL_SPEED, SwerveConstants.MAX_ROTATIONAL_SPEED);
            // Apply omega while preserving last requested translation speeds
            ChassisSpeeds target = new ChassisSpeeds(lastRequestedRobotRelativeSpeeds.vxMetersPerSecond,
                    lastRequestedRobotRelativeSpeeds.vyMetersPerSecond, omega);
            swerveDrive.setChassisSpeeds(target);
        }
    }


    public void drive(double y, double x, double theta, boolean fieldRelative, boolean alignLimelight, boolean resetGyro) {
        if (!swerveEnabled) {
            // Temporarily ignore drive commands while swerve is disabled.
            return;
        }
        ChassisSpeeds newDesiredSpeeds; 
        
        // Centralized rotation handling: apply deadband, scaling, and optionally
        // reduce rotation while translating to avoid propulsion.
        double rotInput = MathUtil.applyDeadband(theta, frc.robot.Constants.OIConstants.DEADBAND);
        rotInput *= frc.robot.Constants.OIConstants.TELEOP_ROTATION_SCALE;

        // If there's significant translational demand, reduce rotation to avoid sudden lateral propulsion.
        double translationMag = Math.hypot(y, x);
        if (translationMag > 0.2) {
            rotInput *= 0.6; // reduce rotation when translating (tunable)
        }

        if (alignLimelight) {
            newDesiredSpeeds = new ChassisSpeeds(y, x, rotInput);
        } else {
            newDesiredSpeeds = new ChassisSpeeds(
                SwerveConstants.MAX_TRANSLATIONAL_SPEED * y,
                SwerveConstants.MAX_TRANSLATIONAL_SPEED * x,
                SwerveConstants.MAX_ROTATIONAL_SPEED * rotInput
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
        if (!swerveEnabled) {
            return new Pose2d();
        }

        return swerveDrive.getPose();
    }

    public ChassisSpeeds getSpeeds() {
        if (!swerveEnabled) {
            return new ChassisSpeeds();
        }

        return swerveDrive.getRobotVelocity();
    }

    public SwerveModulePosition[] getPositions() {
        if (!swerveEnabled) {
            // Return zeroed positions for 4 modules as a safe default
            return new SwerveModulePosition[] {
                new SwerveModulePosition(0.0, new Rotation2d()),
                new SwerveModulePosition(0.0, new Rotation2d()),
                new SwerveModulePosition(0.0, new Rotation2d()),
                new SwerveModulePosition(0.0, new Rotation2d())
            };
        }

        return swerveDrive.getModulePositions();
    } 

    public void resetPose(Pose2d pose) {
        if (!swerveEnabled) {
            return;
        }

        swerveDrive.resetOdometry(pose);
    }

    public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        if (!swerveEnabled) {
            return;
        }

        swerveDrive.addVisionMeasurement(pose, timestampSeconds, stdDevs);
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        if (!swerveEnabled) {
            return;
        }

        Rotation2d drivingAngle = swerveDrive.getOdometryHeading().minus(gyroOffset);
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, drivingAngle));
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        lastRequestedRobotRelativeSpeeds = robotRelativeSpeeds;

        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        if (!swerveEnabled) {
            return;
        }

        swerveDrive.setChassisSpeeds(targetSpeeds);
    }

    public void setStates(SwerveModuleState[] targetStates) {
        if (!swerveEnabled) {
            return;
        }

        swerveDrive.setModuleStates(targetStates, false);
    }


    public SwerveModuleState[] getStates() {
        if (!swerveEnabled) {
            return new SwerveModuleState[] {
                new SwerveModuleState(0.0, new Rotation2d()),
                new SwerveModuleState(0.0, new Rotation2d()),
                new SwerveModuleState(0.0, new Rotation2d()),
                new SwerveModuleState(0.0, new Rotation2d())
            };
        }

        return swerveDrive.getStates();
    }

    public void logStates() {
        if (!swerveEnabled) {
            // Publish placeholder telemetry so dashboards remain readable
            SmartDashboard.putBoolean("Swerve/Diag/RequestedMotion", false);
            SmartDashboard.putBoolean("Swerve/Diag/CommandReachingModules", false);
            SmartDashboard.putNumber("Swerve/Diag/MaxDriveCmdAbs", 0.0);
            SmartDashboard.putNumber("Swerve/Diag/OdometryHeadingDeg", 0.0);
            SmartDashboard.putNumber("Swerve/Diag/RequestedVx", 0.0);
            SmartDashboard.putNumber("Swerve/Diag/RequestedVy", 0.0);
            SmartDashboard.putNumber("Swerve/Diag/RequestedOmegaRadS", 0.0);
            SmartDashboard.putNumber("Swerve/Diag/RobotVx", 0.0);
            SmartDashboard.putNumber("Swerve/Diag/RobotVy", 0.0);
            SmartDashboard.putNumber("Swerve/Diag/RobotOmegaRadS", 0.0);
            for (int i = 0; i < 4; ++i) {
                SmartDashboard.putNumber(String.format("Swerve/Diag/Module%dAngleDeg", i), 0.0);
            }
            SmartDashboard.putNumber("IMU/RawYawDeg", hardwareGyro.getYawDegrees());
            SmartDashboard.putNumber("IMU/RawPitchDeg", hardwareGyro.getPitchDegrees());
            SmartDashboard.putNumber("IMU/RawRollDeg", hardwareGyro.getRollDegrees());
            SmartDashboard.putBoolean("IMU/PitchRollOK", true);
            SmartDashboard.putBoolean("IMU/PitchOrRollExceeded", false);
            return;
        }

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

        // Additional telemetry to help debug orientation/propulsion issues
        double headingDeg = Math.toDegrees(swerveDrive.getOdometryHeading().getRadians());
        SmartDashboard.putNumber("Swerve/Diag/OdometryHeadingDeg", headingDeg);

    // Publish requested chassis speeds vs actual robot velocity for debugging
    SmartDashboard.putNumber("Swerve/Diag/RequestedVx", lastRequestedRobotRelativeSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Swerve/Diag/RequestedVy", lastRequestedRobotRelativeSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Swerve/Diag/RequestedOmegaRadS", lastRequestedRobotRelativeSpeeds.omegaRadiansPerSecond);
    var robotVel = swerveDrive.getRobotVelocity();
    SmartDashboard.putNumber("Swerve/Diag/RobotVx", robotVel.vxMetersPerSecond);
    SmartDashboard.putNumber("Swerve/Diag/RobotVy", robotVel.vyMetersPerSecond);
    SmartDashboard.putNumber("Swerve/Diag/RobotOmegaRadS", robotVel.omegaRadiansPerSecond);

        // Publish individual module angles (degrees) for quick verification
        for (int i = 0; i < currentStates.length; ++i) {
            double moduleAngleDeg = Math.toDegrees(currentStates[i].angle.getRadians());
            SmartDashboard.putNumber(String.format("Swerve/Diag/Module%dAngleDeg", i), moduleAngleDeg);
        }

        // Publish raw IMU axes so we can verify mount pose and behavior during rotation.
        double imuYaw = hardwareGyro.getYawDegrees();
        double imuPitch = hardwareGyro.getPitchDegrees();
        double imuRoll = hardwareGyro.getRollDegrees();
        SmartDashboard.putNumber("IMU/RawYawDeg", imuYaw);
        SmartDashboard.putNumber("IMU/RawPitchDeg", imuPitch);
        SmartDashboard.putNumber("IMU/RawRollDeg", imuRoll);

        // When rotating on the Z axis, pitch and roll should remain near zero.
        double pitchRollThresholdDeg = 5.0; // warn if tilt exceeds 5 degrees
        boolean pitchRollOK = Math.abs(imuPitch) < pitchRollThresholdDeg && Math.abs(imuRoll) < pitchRollThresholdDeg;
        SmartDashboard.putBoolean("IMU/PitchRollOK", pitchRollOK);
        SmartDashboard.putBoolean("IMU/PitchOrRollExceeded", !pitchRollOK);
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
