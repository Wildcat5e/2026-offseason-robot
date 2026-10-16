package frc.robot.subsystems;

import frc.robot.subsystems.Intake;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
    private final TalonFX kickerMotor = new TalonFX(14);
    private final TalonFX conveyorMotor = new TalonFX(15);

    public Hopper() {}

    public void setHopperVoltages(double kickerVoltage, double conveyorVoltage) {
        kickerMotor.setVoltage(kickerVoltage);
        conveyorMotor.setVoltage(conveyorVoltage);
    }

    public double getKickerVoltage() {
        return kickerMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getConveyorVoltage() {
        return conveyorMotor.getMotorVoltage().getValueAsDouble();
    }

    public void setKickerVoltage(double volts) {
        kickerMotor.setVoltage(volts);
    }

    public void setConveyorVoltage(double volts) {
        conveyorMotor.setVoltage(volts);
    }
}
