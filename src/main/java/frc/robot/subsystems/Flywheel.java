package frc.robot.subsystems;

import java.util.Map;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
    private final TalonFX leftFlywheelMotor = new TalonFX(21);
    private final TalonFX rightFlywheelMotor = new TalonFX(20);

    double kickerVoltage = 4;
    double conveyorVoltage = 8;

    private final Hopper hopper = new Hopper();

    public Flywheel() {
        System.out.println("Charlie wuz here");
    }

    public Command spinFlywheel() {
        return startEnd(() -> {
            setFlywheelVoltage(3);
            hopper.setKickerVoltage(kickerVoltage);
            hopper.setConveyorVoltage(conveyorVoltage);
        }, () -> {
            setFlywheelVoltage(0);
            hopper.setKickerVoltage(0);
            hopper.setConveyorVoltage(0);
        });
    }

    private void setFlywheelVoltage(double volts) {
        leftFlywheelMotor.setVoltage(volts);
        rightFlywheelMotor.setVoltage(volts);
    }
}
