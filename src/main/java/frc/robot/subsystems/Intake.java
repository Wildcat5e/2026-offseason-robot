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


    public Command raiseIntake() {
        return startEnd( // @formatter:off
            () -> extenderMotor.setVoltage((1)),
            () -> extenderMotor.setVoltage(0)); // @formatter:on
    }

    public Command lowerIntake() {
        return startEnd( // @formatter:off
            () -> extenderMotor.setVoltage((-1)),
            () -> extenderMotor.setVoltage(0));// @formatter:on
    }

    // get new data, intake pushed too far in
    public Command fullyRaiseIntake() {
        final double tolerance = 0.03;
        return new FunctionalCommand( // @formatter:off
            () -> extenderMotor.setVoltage(1), 
            () -> {},
            (interrupted) -> extenderMotor.setVoltage(0), 
            () -> {
                return (extenderMotor.getPosition().getValueAsDouble() >= -0.05 - tolerance);
            }, this); // @formatter:on
    }

    // get new data, data was when the chain was being bent
    public Command fullyLowerIntake() {
        final double tolerance = 0.01;
        return new FunctionalCommand( // @formatter:off
            () -> extenderMotor.setVoltage(-1), 
            () -> {},
            (interrupted) -> extenderMotor.setVoltage(0), 
            () -> {
                return (extenderMotor.getPosition().getValueAsDouble() <= -0.28076171875 + tolerance);
            }, this); // @formatter:on
    }

    public Command setExtenderPositionZero() {
        return runOnce(() -> extenderMotor.setPosition(0));
    }


    public Command runScooper() {
        return startEnd( // @formatter:off
            () -> scooperMotor.setVoltage(2), 
            () -> scooperMotor.setVoltage(0)); // @formatter:on
    }

    public Command runPusher() {
        return startEnd( // @formatter:off
            () -> pusherMotor.setVoltage(2),
            () -> pusherMotor.setVoltage(0)
        ); // @formatter: on
    }

    
    public Command intakeFuel() {
        return startEnd( //@formatter:off
            () -> setScooperAndPusherVoltages(3),
            () -> setScooperAndPusherVoltages(0)
        ); // @formatter:on
    }

    private void setScooperAndPusherVoltages(double volts) {
        scooperMotor.setVoltage(volts);
        pusherMotor.setVoltage(volts);
    }


    // build method to detect intake jam later, and knowing when to stop the intake, beam break, motor current limits

    @Override
    public void periodic() {}


}
