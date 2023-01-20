package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    /**
     * Creates a new Vision.
     */
    private PhotonPoseEstimator m_poseEstimator;
    private PhotonCamera m_cam;
    private AprilTagFieldLayout m_fieldLayout;
    private FullSwerveBase m_drive;
    private final PoseStrategy m_poseStrategy;
    private Optional<EstimatedRobotPose> curPose;
    private Pose3d curPose3d;

    public Vision(PhotonCamera cam, FullSwerveBase drive) {
        this.m_cam = cam;
        m_drive = drive;
        try {
            this.m_fieldLayout = new AprilTagFieldLayout("taglayout.json");
        } catch (IOException e) {
            System.out.println("Error loading taglayout.json");
            System.exit(1);
        }
        this.m_poseStrategy = PoseStrategy.LOWEST_AMBIGUITY;

        this.m_poseEstimator = new PhotonPoseEstimator(m_fieldLayout, m_poseStrategy,
                cam, new Transform3d());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        PhotonPipelineResult latestResults = this.getLatestResult();
        if (latestResults.hasTargets()) {
            curPose = m_poseEstimator.update();
            if (curPose.get() != null) {
                curPose3d = curPose.get().estimatedPose;
                m_drive.addVisionOdometry(curPose3d);
            }
            // m_drive.addVisionOdometry(latestResults, 0);
            // System.out.println("Target Found");
            // System.out.println("Target Angle: " + latestResults.getBestTarget().getTargetYaw());
            // System.out.println("Target Distance: " + latestResults.getBestTarget().getTargetDistance());
        }
        // System.out.println()
    }

    public PhotonPipelineResult getLatestResult() {
        return m_cam.getLatestResult();
    }

}
