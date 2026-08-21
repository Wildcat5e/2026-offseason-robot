package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RotateToHub;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update
 * the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    public static final double MAX_LINEAR_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_ANGULAR_SPEED = 1.5 * Math.PI;
    static final double DEADZONE = .1;
    EventLoop shooting;
    private final Intake intake = new Intake();
    private final Flywheel flywheel = new Flywheel();

    CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    SwerveRequest.FieldCentric swerveRequest = new SwerveRequest.FieldCentric();
    CommandXboxController controller = new CommandXboxController(0);

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    public Robot() {
        controllerTesting();

    }

    public void controllerTesting() {
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
            swerveRequest.withVelocityY(MathUtil.applyDeadband(-controller.getLeftX(), DEADZONE) * MAX_LINEAR_SPEED);
            swerveRequest.withVelocityX(MathUtil.applyDeadband(-controller.getLeftY(), DEADZONE) * MAX_LINEAR_SPEED);
            return swerveRequest
                .withRotationalRate(MathUtil.applyDeadband(-controller.getRightX(), DEADZONE) * MAX_ANGULAR_SPEED);
        }));

        controller.povUp().whileTrue(intake.IntakeUp());
        controller.povDown().whileTrue(intake.IntakeDown());
        controller.leftTrigger().whileTrue(intake.intakeFuel());
        controller.a().whileTrue(new RotateToHub(drivetrain, flywheel));
        // RotateToHub also shoots the fuel, hold down rightTrigger to manually shoot fuel.
        controller.rightTrigger().whileTrue(flywheel.shootFuel());
        controller.leftBumper().onTrue(new InstantCommand(() -> Flywheel.decreaseMultiplier()));
        controller.rightBumper().onTrue(new InstantCommand(() -> Flywheel.increaseMultiplier()));
        /*
         * TODO: Nicholas suggests that the left and right bumper should be used to raise and lower intake (with
         * assignments opposite to that in the Rebuilt 2026 Robot). The increase and decrease multiplier should instead
         * be assigned to the current keys that the raise and lower intake are set.
         */
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
     * during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard integrated
     * updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /**
     * This function runs when the robot is first started up. Use it for set up tasks like connecting controllers
     */
    @Override
    public void robotInit() {}

    /** This function is called once when auton is enabled. */
    @Override
    public void autonomousInit() {

    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {}

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {}

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {}

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {}

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
