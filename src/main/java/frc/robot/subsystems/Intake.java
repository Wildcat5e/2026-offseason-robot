// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
    /** Creates a new Intake. */

    private final TalonFX pusherMotor = new TalonFX(16);
    private final TalonFX scooperMotor = new TalonFX(18);
    private final TalonFX extenderMotor = new TalonFX(17);


    public Intake() {
        extenderMotor.setPosition(0);

    }


    public Command IntakeUp() {
        return startEnd( // @formatter:off
            () -> extenderMotor.setVoltage((1)),
            () -> extenderMotor.setVoltage(0)); // @formatter:on
    }

    public Command IntakeDown() {
        return startEnd( // @formatter:off
            () -> extenderMotor.setVoltage((-1)),
            () -> extenderMotor.setVoltage(0));// @formatter:on
    }

    // get new data, intake pushed too far in
    public Command fullyIntakeUp() {
        final double tolerance = 0.03;
        return new FunctionalCommand( // @formatter:off
            () -> extenderMotor.setVoltage(0), 
            () -> extenderMotor.setVoltage(1),
            (interrupted) -> extenderMotor.setVoltage(0), 
            () -> {
                return (extenderMotor.getPosition().getValueAsDouble() >= -0.05 - tolerance);
            }, this); // @formatter:on
    }

    // get new data, data was when the chain was being bent
    public Command fullyIntakeDown() {
        final double tolerance = 0.01;
        return new FunctionalCommand( // @formatter:off
            () -> extenderMotor.setVoltage(0), 
            () -> extenderMotor.setVoltage(-1),
            (interrupted) -> extenderMotor.setVoltage(0), 
            () -> {
                return (extenderMotor.getPosition().getValueAsDouble() <= -0.28076171875 + tolerance);
            }, this); // @formatter:on
    }

    public Command setExtenderPositionZero() {
        return runOnce(() -> extenderMotor.setPosition(0));
    }


    public Command intakeFuel() {
        return startEnd( // @formatter:off
            () -> scooperMotor.setVoltage(1), 
            () -> scooperMotor.setVoltage(0)); // @formatter:on
    }

    // build method to detect intake jam later, and knowing when to stop the intake, beam break, motor current limits

    @Override
    public void periodic() {
        System.out.println(extenderMotor.getPosition());
    }


}
