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
    static double flywheelMultiplier = 1;

    // kickerVoltage and conveyorVoltage are stored in here and then passed into Hopper

    Hopper hopper = new Hopper();

    public Command shootFuel() {
        //TODO: Make it so the flywheel only shoots once up to speed (optional because in comp we will just use rotate to hub)
        // Preferably this method would stop using voltage and just use something like setRPM
        return startEnd(() -> {
            leftFlywheelMotor.setVoltage(flywheelVoltage * flywheelMultiplier);
            rightFlywheelMotor.setVoltage(flywheelVoltage * flywheelMultiplier);
            hopper.setKickerVoltage(kickerVoltage);
            hopper.setConveyorVoltage(conveyorVoltage);
        }, () -> {
            leftFlywheelMotor.setVoltage(0);
            rightFlywheelMotor.setVoltage(0);
            hopper.setKickerVoltage(0);
            hopper.setConveyorVoltage(0);
        });
    }

    public static void increaseMultiplier() {
        flywheelMultiplier = flywheelMultiplier + .1;
    }

    public static void decreaseMultiplier() {
        flywheelMultiplier = flywheelMultiplier - .1;
    }

    public void setRPM(double rpmRequest) {
        rpmRequest = rpmRequest * flywheelMultiplier;
        leftFlywheelMotor.setControl(motorVel.withVelocity(rpmRequest / 60));
        rightFlywheelMotor.setControl(motorVel.withVelocity(rpmRequest / 60));
    }
}
