package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class PhotonVision extends SubsystemBase {
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEV = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEV = VecBuilder.fill(0.5, 0.5, 1);
    public static final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final Transform3d robotToCam = new Transform3d(0.5, 0.0, 0.5, new Rotation3d(0, 0, 0));
    public Optional<EstimatedRobotPose> visionEst = Optional.empty();
    private static final Matrix<N3, N1> MAX_STD_DEV =
        VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    private static final Matrix<N3, N1> ZERO_STD_DEV = VecBuilder.fill(0, 0, 0);
    private final EstimateConsumer estConsumer;
    PhotonPoseEstimator photonEstimator;
    PhotonCamera camera;
    // use curStdDevs to set the current standard deviations that you need to use, based on tags, estimated pose
    private Matrix<N3, N1> curStdDevs;

    public PhotonVision(EstimateConsumer estConsumer) {
        this.estConsumer = estConsumer;
        this.camera = new PhotonCamera("Microsoft_LifeCam_HD-3000-Left-USB2-Vert");
        this.photonEstimator = new PhotonPoseEstimator(tagLayout, robotToCam);
    }

    @Override
    public void periodic() {
        curStdDevs = SINGLE_TAG_STD_DEV; // for insurance? clarify with ethan at buildsite
        for (var result : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }

            visionEst.ifPresent(est -> {
                updateEstimationStdDevs(visionEst, result.getTargets());
                System.out.println(curStdDevs);
                estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, curStdDevs);
            });
        }
    }

    /*
     * They call visionEst before vision.ifPresent because visionEst will hold old data if there are no new tags, which
     * is bad that's why they did that in the example
     */
    // tag pose is separate from optional so we need to check?
    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose,
        List<PhotonTrackedTarget> targets) {
        double totalDistance = 0;
        int tagCount = 0;
        for (PhotonTrackedTarget target : targets) {
            Optional<Pose3d> tagPose = tagLayout.getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) {
                continue;
            } else {
                Translation2d robotPose = estimatedPose.get().estimatedPose.toPose2d().getTranslation();
                totalDistance += robotPose.getDistance(tagPose.get().toPose2d().getTranslation()); // in meters
                tagCount++;
            }
        }
        if (tagCount == 0) {
            curStdDevs = MAX_STD_DEV;
            return;
        }
        double averageDistance = totalDistance / tagCount;
        double multiplier = 1 + (Math.pow(averageDistance, 2) / 30.0); // 30 is a placeholder factor for now
        if (tagCount == 1) {
            curStdDevs = SINGLE_TAG_STD_DEV;
            curStdDevs = curStdDevs.times(multiplier);
        } else {
            curStdDevs = MULTI_TAG_STD_DEV;
            curStdDevs = curStdDevs.times(multiplier);
        }

    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
