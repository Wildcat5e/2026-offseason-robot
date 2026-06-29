package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

public class Hopper {
    private final TalonFX kickerMotor = new TalonFX(14);
    private final TalonFX conveyorMotor = new TalonFX(15);

    public void setKickerVoltage(double voltage) {
        kickerMotor.setVoltage(voltage);
    }

    public void setConveyorVoltage(double voltage) {
        conveyorMotor.setVoltage(voltage);
    }
}
