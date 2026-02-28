package frc.robot.subsystems.intake;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.IOException;
import java.net.URL;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTag {
    private static final String OFFICIAL_2026_ANDYMARK_JSON_URL =
            "https://raw.githubusercontent.com/wpilibsuite/allwpilib/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2026-rebuilt-andymark.json";
    private static final String DEFAULT_CAMERA_NAME = "Arducam_OV9782_USB_Camera";

    private final AprilTagFieldLayout fieldLayout;
    private final PhotonCamera camera;

    public AprilTag() {
        this(DEFAULT_CAMERA_NAME);
    }

    public AprilTag(String cameraName) {
        fieldLayout = loadFieldLayout();
        camera = new PhotonCamera(cameraName);
    }

    private AprilTagFieldLayout loadFieldLayout() {
        try {
            return new ObjectMapper().readValue(new URL(OFFICIAL_2026_ANDYMARK_JSON_URL), AprilTagFieldLayout.class);
        } catch (IOException e) {
            DriverStation.reportWarning(
                    "Unable to load 2026 AndyMark AprilTag layout from URL. Falling back to WPILib default field: "
                            + e.getMessage(),
                    false);
            return AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        }
    }

    public AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }

    public void updateOriginFromAlliance(Optional<DriverStation.Alliance> alliance) {
        OriginPosition origin = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red
                ? OriginPosition.kRedAllianceWallRightSide
                : OriginPosition.kBlueAllianceWallRightSide;
        fieldLayout.setOrigin(origin);
    }

    public Optional<Pose3d> getTagPose3d(int tagId) {
        return fieldLayout.getTagPose(tagId);
    }

    public Optional<Pose2d> getTagPose2d(int tagId) {
        return getTagPose3d(tagId).map(Pose3d::toPose2d);
    }

    public Optional<PhotonTrackedTarget> getLatestTrackedTarget(int tagId) {
        PhotonPipelineResult result = camera.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        return targets.stream()
                .filter(target -> target.getFiducialId() == tagId)
                .max(Comparator.comparingDouble(PhotonTrackedTarget::getArea));
    }

    public Optional<PhotonTrackedTarget> getBestVisibleTarget() {
        PhotonPipelineResult result = camera.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        return targets.stream().max(Comparator.comparingDouble(PhotonTrackedTarget::getArea));
    }

    public Optional<Transform3d> getCameraToTagTransform(int tagId) {
        return getLatestTrackedTarget(tagId).map(PhotonTrackedTarget::getBestCameraToTarget);
    }

    public boolean seesTag(int tagId) {
        return getLatestTrackedTarget(tagId).isPresent();
    }

    public void logVision(int tagId) {
        Optional<PhotonTrackedTarget> target = getLatestTrackedTarget(tagId);
        SmartDashboard.putBoolean("Vision/Tag" + tagId + "/Visible", target.isPresent());
        target.ifPresentOrElse(
                t -> {
                    SmartDashboard.putNumber("Vision/Tag" + tagId + "/YawDeg", t.getYaw());
                    SmartDashboard.putNumber("Vision/Tag" + tagId + "/PitchDeg", t.getPitch());
                    SmartDashboard.putNumber("Vision/Tag" + tagId + "/Area", t.getArea());
                },
                () -> {
                    SmartDashboard.putNumber("Vision/Tag" + tagId + "/YawDeg", 0.0);
                    SmartDashboard.putNumber("Vision/Tag" + tagId + "/PitchDeg", 0.0);
                    SmartDashboard.putNumber("Vision/Tag" + tagId + "/Area", 0.0);
                });
    }
}
