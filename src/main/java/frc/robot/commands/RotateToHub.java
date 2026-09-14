// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RotateToHub extends Command {
    private static final double MAX_ROTATION_RATE = 1.5 * Math.PI;

    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController pidController = new PIDController(3, 0, 0);
    private final FieldCentric driveRequest = new FieldCentric();
    private double measuredHeading;

    public RotateToHub(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        pidController.enableContinuousInput(-Math.PI, Math.PI);
        pidController.setTolerance(Math.toRadians(0.5));
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        pidController.reset();
        measuredHeading = drivetrain.getState().Pose.getRotation().getRadians();
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getState().Pose;
        Rotation2d targetRotation = getTargetRotation(robotPose);
        double rotationSpeed = pidController.calculate(measuredHeading, targetRotation.getDegrees());
        rotationSpeed = MathUtil.clamp(rotationSpeed, -MAX_ROTATION_RATE, MAX_ROTATION_RATE);

        drivetrain.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(rotationSpeed));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public static Translation2d getHubCoordinates() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
            ? Constants.RED_HUB_COORDINATES
            : Constants.BLUE_HUB_COORDINATES;
    }

    private static Rotation2d getTargetRotation(Pose2d robotPose) {
        Translation2d toHubVector = getHubCoordinates().minus(robotPose.getTranslation());
        return toHubVector.getAngle();
    }
}
