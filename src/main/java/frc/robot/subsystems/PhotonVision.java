package frc.robot.subsystems;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
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
        for (var result : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }
            // updateEstimationStdDevs(visionEst, result.getTargets());

            // if (Robot.isSimulation()) {
            //     visionEst.ifPresentOrElse(
            //         est -> 
            //             getSimDebugField()
            //             .getObject("VisionEstimation")
            //             .setPose(est.estimatedPose.toPose2d()),
            //             () -> {
            //                 getSimDebugField().getObject("VisionEstimation").setPoses();
            //             });

            // }

            visionEst.ifPresent(est -> {
                var estStdDevs = SINGLE_TAG_STD_DEV;
                estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            });
        }
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
