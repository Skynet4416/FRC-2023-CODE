package frc.robot.subsystems.Vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision.AprilTag;
import frc.robot.Constants.Vision.ReflectiveTape;

public class VisionSubsystem extends SubsystemBase {
    private PhotonCamera m_aprilTagCamera;
    private PhotonCamera m_reflectiveTapeCamera;
    private AprilTagFieldLayout m_fieldLayout = null;
    private final Transform3d m_CamPositionAtRobot = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(0, 0, 0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
                                      // from center.
    private PhotonPoseEstimator m_photonPoseEstimator = null;
    private boolean enable;

    public VisionSubsystem(boolean enable) {
        this.enable = enable;
        if (this.enable) {
            m_aprilTagCamera = new PhotonCamera(AprilTag.kCameraName);
            m_reflectiveTapeCamera = new PhotonCamera(AprilTag.kCameraName);
            try {
                m_fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile);
                m_photonPoseEstimator = new PhotonPoseEstimator(m_fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                        m_aprilTagCamera, m_CamPositionAtRobot);
            } catch (IOException exception) {
                System.out.println(exception);
            }

        }

    }

    public boolean aprilTagHasTarget() {
        if (enable) {
            return m_aprilTagCamera.getLatestResult().hasTargets();

        }
        return false;
    }

    public boolean reflectiveHasTarget() {
        if (enable) {
            return m_reflectiveTapeCamera.getLatestResult().hasTargets();

        }
        return false;
    }

    public boolean hasTarget() {
        return reflectiveHasTarget() || aprilTagHasTarget();
    }

    public PhotonTrackedTarget getReflectiveTarget() {
        if (reflectiveHasTarget()) {
            return m_reflectiveTapeCamera.getLatestResult().getBestTarget();
        }
        return null;
    }

    public PhotonTrackedTarget getAprilTarget() {
        if (aprilTagHasTarget()) {
            return m_aprilTagCamera.getLatestResult().getBestTarget();
        }
        return null;
    }

    public PhotonTrackedTarget getVisionTarget() {

        if (reflectiveHasTarget()) {
            return getReflectiveTarget();
        } else if (aprilTagHasTarget()) {
            return getAprilTarget();
        }
        return null;

    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (enable) {
            m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
            return m_photonPoseEstimator.update();
        }
        return null;
        
    }

}
