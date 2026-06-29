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
        return startEnd(() -> setFlywheelVoltage(3), () -> setFlywheelVoltage(0));
    }

    public void spinFlywheel(double volts) {
        setFlywheelVoltage(volts);
    }

    private void setFlywheelVoltage(double volts) {
        leftFlywheelMotor.setVoltage(volts);
        rightFlywheelMotor.setVoltage(volts);
    }
    /*
     * My logic is to write a method to check if flywheel is up to speed, to do this, we need to first be able to get
     * the robot's position on the field, then go to the interpolation data for the speed that we need, and then start
     * running the flywheel until isFlywheelUpToSpeed() returns true Uncomment the method below when you are ready to
     * write it, leaving it commented out so it compiles for now
     */
    //public boolean isFlywheelUpToSpeed() {}

}
