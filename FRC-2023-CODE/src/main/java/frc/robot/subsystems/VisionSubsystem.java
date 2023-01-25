package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.Vision.AprilTag;
import frc.robot.Constants.Vision.ReflectiveTape;

public class VisionSubsystem {
    private final PhotonCamera m_aprilTagCamera = new PhotonCamera(AprilTag.kCameraName);
    private final PhotonCamera m_reflectiveTapeCamera = new PhotonCamera(ReflectiveTape.kReflectiveTapeCameraName);
    private AprilTagFieldLayout m_fieldLayout = null;
    private final Transform3d m_CamPositionAtRobot = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(0, 0, 0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
                                      // from center.
    private PhotonPoseEstimator m_photonPoseEstimator = null;

    public VisionSubsystem() {
        try {
            m_fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile);
            m_photonPoseEstimator = new PhotonPoseEstimator(m_fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                    m_aprilTagCamera, m_CamPositionAtRobot);
        } catch (IOException exception) {

            System.out.println(exception);
        }
    }

    public boolean aprilTagHasTarget() {
        return m_aprilTagCamera.getLatestResult().hasTargets();
    }

    public boolean reflectiveHasTarget() {
        return m_aprilTagCamera.getLatestResult().hasTargets();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return m_photonPoseEstimator.update();
    }

    
    
}
