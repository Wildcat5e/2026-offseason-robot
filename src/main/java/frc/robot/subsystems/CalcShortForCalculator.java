package frc.robot.subsystems;

import java.util.Map;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class CalcShortForCalculator {
    static double sq(double input) {
        return input * input;
    }

    static Translation2d BLUE_HUB_POSITION = new Translation2d(4.625, 4.03);
    static Translation2d RED_HUB_POSITION = new Translation2d(11.915, 4.03);
    static boolean isBlueAlliance;
    static Translation2d targetHubPosition;
    static double HOOD_ANGLE_RADIANS = Math.toRadians(64);
    static double flywheelMultiplyer = 0;
    //TODO: find the correct flywheelMultiplyer

    static Translation2d getTargetHubPosition() {
        DriverStation.getAlliance().ifPresent(fms_alliance -> isBlueAlliance = fms_alliance == Alliance.Blue);
        targetHubPosition = isBlueAlliance ? BLUE_HUB_POSITION : RED_HUB_POSITION;
        return targetHubPosition;
    }

    //returns a feild angle
    static double angleToHub(CommandSwerveDrivetrain drivetrain) {
        DriverStation.getAlliance().ifPresent(fms_alliance -> isBlueAlliance = fms_alliance == Alliance.Blue);
        Pose2d currentPose = drivetrain.getState().Pose;

        // ternary operator
        targetHubPosition = isBlueAlliance ? BLUE_HUB_POSITION : RED_HUB_POSITION;

        return Math.atan2((targetHubPosition.getY() - currentPose.getY()),
            (targetHubPosition.getX() - currentPose.getX()));
    }

    static double distanceSq(CommandSwerveDrivetrain drivetrain) {
        DriverStation.getAlliance().ifPresent(fms_alliance -> isBlueAlliance = fms_alliance == Alliance.Blue);
        Pose2d currentPose = drivetrain.getState().Pose;

        targetHubPosition = isBlueAlliance ? BLUE_HUB_POSITION : RED_HUB_POSITION;

        double deltaX = targetHubPosition.getX() - currentPose.getX();
        double deltaY = targetHubPosition.getY() - currentPose.getY();

        return sq(deltaX) + sq(deltaY);
    }

    static double distance(CommandSwerveDrivetrain drivetrain) {
        DriverStation.getAlliance().ifPresent(fms_alliance -> isBlueAlliance = fms_alliance == Alliance.Blue);
        Pose2d currentPose = drivetrain.getState().Pose;

        targetHubPosition = isBlueAlliance ? BLUE_HUB_POSITION : RED_HUB_POSITION;

        double deltaX = targetHubPosition.getX() - currentPose.getX();
        double deltaY = targetHubPosition.getY() - currentPose.getY();

        return Math.sqrt(sq(deltaX) + sq(deltaY));
    }

    static double getFlywheelVel(CommandSwerveDrivetrain drivetrain) {
        double distanceToHub = distance(drivetrain);
        return flywheelSpeeds.get(distanceToHub);
    }

    static InterpolatingDoubleTreeMap flywheelSpeeds = InterpolatingDoubleTreeMap.ofEntries(
    // @formatter:off
        // Distance (M), Flywheel RPM
        Map.entry(1.78, 46.5),
        Map.entry(1.98, 47.5),
        Map.entry(2.20, 49.0),
        Map.entry(2.40, 50.0),
        Map.entry(2.60, 51.0),
        Map.entry(2.80, 52.5),
        Map.entry(3.00, 54.0),
        Map.entry(3.19, 56.0),
        Map.entry(3.40, 57.0),
        Map.entry(3.58, 58.0),
        Map.entry(3.80, 59.0),
        Map.entry(4.00, 62.5),
        Map.entry(4.20, 65.0),
        Map.entry(4.50, 66.0),
        Map.entry(4.85, 69.0),
        Map.entry(4.91, 72.4),
        Map.entry(5.02, 73.2),
        Map.entry(5.18, 74.8));
    // @formatter:on
}
