// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    /** Creates a new Intake. */

    private final TalonFX pusherMotor = new TalonFX(16);
    private final TalonFX scooperMotor = new TalonFX(18);
    private final TalonFX extenderMotor = new TalonFX(17);


    public Intake() {
        extenderMotor.setPosition(0);

    }

    // public Command bringIntakeUp() {

    // }
    public Command IntakeUp() {
        return startEnd(() -> extenderMotor.setVoltage((1)), () -> extenderMotor.setVoltage(0));
    }

    public Command IntakeDown() {
        return startEnd(() -> extenderMotor.setVoltage((-1)), () -> extenderMotor.setVoltage(0));
    }


    @Override
    public void periodic() {
        System.out.println(extenderMotor.getPosition());
    }
}
