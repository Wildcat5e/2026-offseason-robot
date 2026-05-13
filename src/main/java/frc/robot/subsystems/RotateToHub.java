package frc.robot.subsystems;

import java.lang.ModuleLayer.Controller;
import java.util.Map;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Robot;

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

        double feedForwardVel = CalcShortForCalculator.getFeedforwardVel(drivetrain);

        drivetrain.setControl(swerveRequest.withRotationalRate(/* cappedVelocity */));
    }

    @Override
    public void end(boolean interrupted) {

    }
}
