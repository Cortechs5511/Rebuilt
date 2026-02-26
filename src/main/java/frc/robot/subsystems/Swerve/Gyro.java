package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class Gyro {
    // If robot-forward is not gyro-zero, set the install offset here.
    private static final double INITIAL_OFFSET_DEGREES = 45.0;

    private final Pigeon2 pigeon;

    public Gyro() {
        pigeon = new Pigeon2(0);
    }

    public Rotation2d getRotation2d() {
        double yawDegrees = pigeon.getYaw().getValueAsDouble() + INITIAL_OFFSET_DEGREES;
        return Rotation2d.fromDegrees(yawDegrees);
    }

    public void resetGyro(double degrees) {
        // Compensate for read-time offset so callers can set true heading directly.
        pigeon.setYaw(degrees - INITIAL_OFFSET_DEGREES);
    }
}
