package frc.robot.Constants;

import java.util.Map;
import java.util.Optional;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {
    private Constants() {}

    public static final Optional<Alliance> alliance = DriverStation.getAlliance();;
    public static final Translation2d RED_HUB_COORDINATES = new Translation2d(11.921, 4.024);
    public static final Translation2d BLUE_HUB_COORDINATES = new Translation2d(4.624, 4.024);

    /*
     * Include field layout eventually and other constant values?
     */

    public static final InterpolatingDoubleTreeMap flywheelSpeeds = InterpolatingDoubleTreeMap.ofEntries(
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
