// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.signals.DiffPIDRefSlopeECUTime_ClosedLoopModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class RotateToHub extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController pidController;
    private final FieldCentric driveRequest = new FieldCentric();

    /** Creates a new RotateToHub. */
    public RotateToHub(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.pidController = new PIDController(3, 0, 0); // placeholder values
        this.pidController.enableContinuousInput(-Math.PI, Math.PI); // look into this
        this.pidController.setTolerance(.5); // what unit
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getState().Pose;
        Rotation2d targetRotation = getTargetRotation(robotPose);
        System.out.println("Robot Pose: " + robotPose);
        double rotationSpeed =
            pidController.calculate(robotPose.getRotation().getRadians(), targetRotation.getRadians());
        System.out.println("Current Rotation: " + robotPose.getRotation().getDegrees());
        System.out.println("Target Rotation: " + targetRotation.getDegrees());
        System.out.println("Rotation speed: " + rotationSpeed); // for debugging
        drivetrain.setControl(driveRequest.withRotationalRate(rotationSpeed));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(driveRequest.withRotationalRate(0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    private static Rotation2d getTargetRotation(Pose2d robotPose) {
        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d hubTranslation = new Translation2d(4.634, 4.041); // hub coordinates from path planner
        Translation2d toHubVector = hubTranslation.minus(robotTranslation); // figure out exactly how this works
        Rotation2d targetRotation = toHubVector.getAngle();
        return targetRotation;
    }
}
