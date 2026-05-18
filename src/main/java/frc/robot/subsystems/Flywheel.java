package frc.robot.subsystems;

import java.util.Map;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Flywheel extends SubsystemBase {
    private final TalonFX leftFlywheelMotor = new TalonFX(21);
    private final TalonFX rightFlywheelMotor = new TalonFX(20);

    double flywheelVoltage = 2;
    double kickerVoltage = 4;
    double conveyorVoltage = 8;
    // kickerVoltage and conveyorVoltage are stored in here and then passed into Hopper

    Hopper hopper = new Hopper();

    public Command shootFuel() {
        return startEnd(() -> {
            leftFlywheelMotor.setVoltage(flywheelVoltage);
            rightFlywheelMotor.setVoltage(-flywheelVoltage);
            // TODO: Check if the flywheel voltage needs to be inverted like this or not
            hopper.setKickerVoltage(kickerVoltage);
            hopper.setConveyorVoltage(conveyorVoltage);
        }, () -> {
            leftFlywheelMotor.setVoltage(0);
            rightFlywheelMotor.setVoltage(0);
            hopper.setKickerVoltage(0);
            hopper.setConveyorVoltage(0);
        });
    }

    public void setFlywheelVel(double targetFlywheelSpeed) {
        //TODO: Replace this placeholder so that RotateToHub can actually shoot 
    }

}
