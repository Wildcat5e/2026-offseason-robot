package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;


public class RotateToHub extends Command {
    final PIDController PID_CONTROLLER = new PIDController(1.0, 0.2, 1.0);
    //TODO: Tune kp (should be lower)
    private Pose2d robotPose;
    SwerveRequest.FieldCentric swerveRequest = new SwerveRequest.FieldCentric();
    private final CommandSwerveDrivetrain drivetrain;
    private final Flywheel flywheel;

    public RotateToHub(CommandSwerveDrivetrain drivetrain, Flywheel flywheel) {
        this.drivetrain = drivetrain;
        this.flywheel = flywheel;

        addRequirements(drivetrain, flywheel);
    }

    @Override
    public void initialize() {
        PID_CONTROLLER.reset();
    }

    @Override
    public void execute() {
        robotPose = drivetrain.getState().Pose;
        double neededAngle = CalcShortForCalculator.angleToHub(drivetrain);
        double headingError = neededAngle - robotPose.getRotation().getRadians();
        double rotationSpeed = PID_CONTROLLER.calculate(headingError);
        drivetrain.setControl(swerveRequest.withRotationalRate(rotationSpeed));

        flywheel.setRPM(CalcShortForCalculator.getFlywheelVel(drivetrain));
        // TODO: Why doesnt the above line run? During testing, the flywheel never spun (perhaps due to the PID overshoot)
    }

    @Override
    public void end(boolean interrupted) {

    }
}
