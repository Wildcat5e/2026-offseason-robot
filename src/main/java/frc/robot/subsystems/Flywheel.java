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

    private void setFlywheelVoltage(double volts) {
        leftFlywheelMotor.setVoltage(volts);
        rightFlywheelMotor.setVoltage(volts);
    }
}
