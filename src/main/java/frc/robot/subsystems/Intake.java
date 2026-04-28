package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intake implements Subsystem {
    private final TalonFX pusherMotor = new TalonFX(16);
    private final TalonFX scooperMotor = new TalonFX(18);
    double pusherVoltage = 1;
    double scooperVoltage = 1;
    // replace with real values from on 2026 rebuilt robot if you deploy

    private final TalonFX extenderMotor = new TalonFX(17);
    double extenderVoltage = 1;
    double retractorVoltage = 1;
    // reminder to change these voltages to reflect the negative and positive voltages
    // on the real robot.

    public Command spin() {
        return startEnd(() -> {
            pusherMotor.setVoltage(pusherVoltage);
            scooperMotor.setVoltage(scooperVoltage);
        }, () -> {
            pusherMotor.setVoltage(0);
            scooperMotor.setVoltage(0);
        });
    }

    public Command extendIntake() {
        return startEnd(() -> extenderMotor.setVoltage(extenderVoltage), () -> extenderMotor.setVoltage(0));
    }

    public Command retractIntake() {
        return startEnd(() -> extenderMotor.setVoltage(retractorVoltage), () -> extenderMotor.setVoltage(0));
    }
}
