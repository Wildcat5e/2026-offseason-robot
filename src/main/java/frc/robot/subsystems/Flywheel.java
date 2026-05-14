package frc.robot.subsystems;

import java.util.Map;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
    private final TalonFX leftFlywheelMotor = new TalonFX(21);
    private final TalonFX rightFlywheelMotor = new TalonFX(20);

    public Flywheel() {}

    public Command spinFlywheel() {
        return startEnd(() -> setFlywheelVoltage(0), () -> setFlywheelVoltage(0));
    }

    private void setFlywheelVoltage(double volts) {
        leftFlywheelMotor.setVoltage(volts);
        rightFlywheelMotor.setVoltage(volts);
    }
    /*
     * My logic is to write a method to check if flywheel is up to speed, to do this, we need to first be able to get
     * the robot's position on the field, then go to the interpolation data for the voltage that we need, and then start
     * running the flywheel until isFlywheelUpToSpeed() returns true Uncomment the method below when you are ready to
     * write it, leaving it commented out so it compiles for now
     */
    //public boolean isFlywheelUpToSpeed() {}

    private static InterpolatingDoubleTreeMap flywheelSpeeds = InterpolatingDoubleTreeMap.ofEntries(
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
}
