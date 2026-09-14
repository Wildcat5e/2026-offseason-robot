// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final TalonFX pusherMotor = new TalonFX(16);
    private final TalonFX scooperMotor = new TalonFX(18);
    private final TalonFX extenderMotor = new TalonFX(17);

    public Intake() {
        extenderMotor.setPosition(0);
    }

    public Command raiseIntake() {
        return startEnd(() -> extenderMotor.setVoltage(1), () -> extenderMotor.setVoltage(0));
    }

    public Command lowerIntake() {
        return startEnd(() -> extenderMotor.setVoltage(-1), () -> extenderMotor.setVoltage(0));
    }

    public Command fullyRaiseIntake() {
        final double tolerance = 0.03;
        return new FunctionalCommand(() -> extenderMotor.setVoltage(1), () -> {},
            interrupted -> extenderMotor.setVoltage(0),
            () -> extenderMotor.getPosition().getValueAsDouble() >= -0.05 - tolerance, this);
    }

    public Command fullyLowerIntake() {
        final double tolerance = 0.01;
        return new FunctionalCommand(() -> extenderMotor.setVoltage(-1), () -> {},
            interrupted -> extenderMotor.setVoltage(0),
            () -> extenderMotor.getPosition().getValueAsDouble() <= -0.28076171875 + tolerance, this);
    }

    public Command setExtenderPositionZero() {
        return runOnce(() -> extenderMotor.setPosition(0));
    }

    public Command runScooper() {
        return startEnd(() -> scooperMotor.setVoltage(2), () -> scooperMotor.setVoltage(0));
    }

    public Command runPusher() {
        return startEnd(() -> pusherMotor.setVoltage(2), () -> pusherMotor.setVoltage(0));
    }

    public Command intakeFuel() {
        return startEnd(() -> setScooperAndPusherVoltages(5), () -> setScooperAndPusherVoltages(0));
    }

    private void setScooperAndPusherVoltages(double volts) {
        scooperMotor.setVoltage(volts);
        pusherMotor.setVoltage(volts);
    }

    @Override
    public void periodic() {}
}
