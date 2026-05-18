package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import frc.robot.Robot;
import frc.robot.subsystems.CalcShortForCalculator.ShootingInfo;

public class RotateToHub extends Command {
    final PIDController PID_CONTROLLER = new PIDController(5.0, 0, 0);
    private Pose2d robotPose;
    CommandSwerveDrivetrain drivetrain;
    SwerveRequest.FieldCentric swerveRequest = new SwerveRequest.FieldCentric();

    @Override
    public void initialize() {
        PID_CONTROLLER.reset();
    }

    @Override
    public void execute() {
        robotPose = drivetrain.getState().Pose;
        double neededAngle = CalcShortForCalculator.theta(drivetrain);
        ShootingInfo shotInfo = CalcShortForCalculator.shootingInfo(drivetrain);
        DoubleSupplier flywheelSpeedMultiplier;
        // TODO: Find out what the correct value for flywheelSpeedMultiplier is
        double targetRPM = shotInfo.targetFlywheelSpeed();
        double targetHeading = shotInfo.targetRobotHeading();

        double feedForwardVel = CalcShortForCalculator.getRobotVector(drivetrain);
        double pidVel = PID_CONTROLLER.calculate(robotPose.getRotation().getRadians(), targetHeading);
        double totalVel = feedForwardVel + pidVel;
        double cappedVel = Math.max(Math.min(totalVel, Robot.MAX_ANGULAR_SPEED), -Robot.MAX_ANGULAR_SPEED);
        rotate(cappedVel);
    }

    @Override
    public void end(boolean interrupted) {

    }

    public void rotate(double cappedVel) {
        drivetrain.setControl(swerveRequest.withRotationalRate(cappedVel));
        PPHolonomicDriveController.overrideRotationFeedback(() -> cappedVel);
        // TODO: ask Ethan what this line does
    }
}
