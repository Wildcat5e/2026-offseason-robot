package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Outtake implements Subsystem {
    private final TalonFX leftFlywheelMotor = new TalonFX(21);
    private final TalonFX rightFlywheelMotor = new TalonFX(20);
    private final TalonFX kickerMotor = new TalonFX(14);
    private final TalonFX conveyorMotor = new TalonFX(15);
    double flywheelVoltage = 2;
    double kickerVoltage = 4;
    double conveyorVoltage = 8;
    // replace with real values from on 2026 rebuilt robot if you deploy

    public Command shoot() {
        return startEnd(() -> {
            leftFlywheelMotor.setVoltage(flywheelVoltage);
            rightFlywheelMotor.setVoltage(-flywheelVoltage);
            kickerMotor.setVoltage(kickerVoltage);
            conveyorMotor.setVoltage(conveyorVoltage);
        }, () -> {
            leftFlywheelMotor.setVoltage(0);
            rightFlywheelMotor.setVoltage(0);
            kickerMotor.setVoltage(0);
            conveyorMotor.setVoltage(0);
        });
    }
}
