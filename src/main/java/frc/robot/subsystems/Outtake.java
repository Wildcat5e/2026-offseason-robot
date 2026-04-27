package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Outtake {
    private final TalonFX leftFlywheelMotor = new TalonFX(21);
    private final TalonFX rightFlywheelMotor = new TalonFX(20);
    private final TalonFX kickerMotor = new TalonFX(14);
    private final TalonFX conveyorMotor = new TalonFX(15);

    public Command shoot() {
        return new InstantCommand();
    }
}
