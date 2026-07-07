package frc.robot.Constants;

import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {
    private Constants() {}

    public static final Optional<Alliance> alliance = DriverStation.getAlliance();;

}
