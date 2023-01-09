package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    /**
     * Creates a new Vision.
     */
    private RobotPoseEstimator m_poseEstimator;
    private PhotonCamera m_cam;
    private AprilTagFieldLayout m_fieldLayout;
    private final PoseStrategy m_poseStrategy;

    public Vision(PhotonCamera cam) {
        this.m_cam = cam;
        try {
            this.m_fieldLayout = new AprilTagFieldLayout("taglayout.json");
        } catch (IOException e) {
            System.out.println("Error loading taglayout.json");
            System.exit(1);
        }
        this.m_poseStrategy = PoseStrategy.LOWEST_AMBIGUITY;

        this.m_poseEstimator = new RobotPoseEstimator(m_fieldLayout, m_poseStrategy,
                List.of(Pair.of(cam, new Transform3d())));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        PhotonPipelineResult latestResults = this.getLatestResult();
        if (latestResults.hasTargets()) {
            m_poseEstimator.update();
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
