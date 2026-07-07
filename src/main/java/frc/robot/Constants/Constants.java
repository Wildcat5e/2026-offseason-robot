package frc.robot.Constants;

import java.util.Optional;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {
    private Constants() {}

    public static final Optional<Alliance> alliance = DriverStation.getAlliance();;
    public static final Translation2d RED_HUB_COORDINATES = new Translation2d(11.921, 4.024);
    public static final Translation2d BLUE_HUB_COORDINATES = new Translation2d(4.624, 4.024);
}
