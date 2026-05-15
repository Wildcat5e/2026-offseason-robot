package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class RotateToHub {
    static Translation2d BLUE_HUB_POSITION = new Translation2d(4.625, 4.03);
    static Translation2d RED_HUB_POSITION = new Translation2d(11.915, 4.03);
    static boolean isBlueAlliance;
    static Translation2d targetHubPosition;

    static double calc(Drivetrain drivetrain, Translation2d target) {
        DriverStation.getAlliance().ifPresent(fms_alliance -> isBlueAlliance = fms_alliance == Alliance.Blue);
        Pose2d currentPose = drivetrain.getState().Pose;

        // ternary operator
        targetHubPosition = isBlueAlliance ? BLUE_HUB_POSITION : RED_HUB_POSITION;

        return Math.atan2((targetHubPosition.getY() - currentPose.getY()),
            (targetHubPosition.getX() - currentPose.getX()));
    }
}
