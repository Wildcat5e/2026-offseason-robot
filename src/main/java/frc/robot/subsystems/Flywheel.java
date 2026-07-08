package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
    private final TalonFX leftFlywheelMotor = new TalonFX(21);
    private final TalonFX rightFlywheelMotor = new TalonFX(20);
    private final VelocityVoltage motorVel = new VelocityVoltage(0);

    double flywheelVoltage = .5;
    double kickerVoltage = -4;
    double conveyorVoltage = -8;
    // kickerVoltage and conveyorVoltage are stored in here and then passed into Hopper

    Hopper hopper = new Hopper();

    public Command shootFuel() {
        return startEnd(() -> {
            leftFlywheelMotor.setVoltage(flywheelVoltage);
            rightFlywheelMotor.setVoltage(flywheelVoltage);
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

    public void setRPM(double rpmRequest) {
        leftFlywheelMotor.setControl(motorVel.withVelocity(rpmRequest / 60));
        rightFlywheelMotor.setControl(motorVel.withVelocity(rpmRequest / 60));
    }
}
