# FRC debugging workshop — instructor answer key

There are exactly seven deliberate differences between the student and corrected source. Four files contain them. The corrected source is a workshop baseline, not a claim that the entire robot has been commissioned.

Distribute only the student folder and student handout initially. Keep this key, the answer-key folder, and instructor-diff.patch private until review. The top-level ZIP includes the answers and should not be given to students intact.

## Defect map and verification

| Bug | Exact location | Cause and fix | Verification and lesson |
|---|---|---|---|
| 1 — compilation | ShootFuel.initialize(), hopper start call | setHopperVoltage does not exist. Use setHopperVoltages(-8, -3). | Rebuild the existing project; the unresolved-method error should disappear. Match a call to its actual declaration, rather than renaming the correct API to match a typo. |
| 2 — comparison | Intake.fullyRaiseIntake(), completion supplier | <= is reversed. Use >= -0.05 - tolerance. | Substitute positions -0.20, -0.08, -0.06: expected false, true, true. Check that the end callback requests 0 V. Direction of travel determines the comparison. |
| 3 — units | RotateToHub.execute(), second PID argument | The target is supplied in degrees while the measurement and wrap range use radians. Use targetRotation.getRadians(). | With a fresh measurement of 0 and target 45 degrees, inputs are 0 and pi/4, giving P output about +2.356 rad/s before/after the cap. Use nonzero targets so this mismatch cannot hide. |
| 4 — stale state | RotateToHub.execute(), immediately after obtaining robotPose | measuredHeading is assigned only in initialize(). Add measuredHeading = robotPose.getRotation().getRadians(); every execute(). | With fixed target 45 degrees and actual headings 0, 15, 30, 45 degrees, expect requested rates approximately 2.356, 1.571, 0.785, 0 rad/s after fixing bug 3. Reading a new object does not refresh a previously copied primitive. |
| 5 — requirements | ShootFuel constructor | addRequirements lists flywheel only. Use addRequirements(flywheel, hopper). | Inspect new ShootFuel(flywheel, hopper).getRequirements(): it must contain both supplied instances. In a controlled scheduler check, a normally interruptible shooting command must be interrupted by a new hopper-requiring command. Requirements arbitrate ownership; they do not physically stop motors by themselves. |
| 6 — cleanup | Intake.intakeFuel(), second startEnd callback | The ending callback is empty. Replace it with () -> setScooperAndPusherVoltages(0). | Trace/log command initialization and cancellation: both requests should go 5 V -> 0 V. Check interruption as well as the second Y press. The motor may coast after zero output is requested. |
| 7 — configuration | Flywheel.rightFlywheelMotor initializer | Both objects use ID 21. The supplied wiring map identifies the right motor as 20. Use new TalonFX(20). | Compare both constructor values against the map: left 21, right 20. Two software objects pointing at the same CAN ID do not create two separate physical controllers. |

For bugs 3 and 4, test one defect at a time with the other corrected. Their effects overlap in the final turn output. For the stale-state check, hold the robot's translation fixed and change its measured heading in the fixture; otherwise the target angle could change too.

Additional aiming checks: with current heading +179 degrees and target -179 degrees, the short error is +2 degrees and the P request is about +0.105 rad/s. With the blue hub at (4.624, 4.024), robot position (4.624, 3.024) produces a +90-degree target; (5.624, 4.024) produces a 180-degree target. Avoid the degenerate fixture where robot and hub share the same coordinates.

These arithmetic checks were evaluated while preparing the packet. Student/corrected files were compared to confirm the seven tracked changes and six identical support files. A real WPILib/CTRE build and robot/scheduler execution were NOT run: the generated TunerConstants, Gradle/vendor configuration, and deploy assets were not provided. Build both overlays in your existing project before the workshop. The student build should fail on bug 1; once corrected, it should proceed with that same dependency set. Treat unrelated dependency/configuration failures as setup problems, not extra assigned defects.

## Changes shared by both versions

These are baseline adjustments relative to the pasted code, not additional student bugs:

- RotateToHub reads DriverStation.getAlliance() when choosing the hub, with a blue fallback. Constants no longer freezes alliance at class initialization.
- Aiming resets PID state on each scheduling, explicitly requests zero translation, and caps the rotational request to the same 1.5*pi rad/s magnitude as the robot's existing angular-speed constant. The cap is not a certification of a safe physical test speed.
- Robot stores/cancels its selected autonomous command on teleop entry and cancels scheduled commands on test entry.
- Unused imports and comments were trimmed in several handwritten files; PhotonVision's estimation logic was preserved. CommandSwerveDrivetrain is reproduced from the attachment, with newline normalization only. Its correct filename includes 'Swerve'.

The supplied motor voltages, hub coordinates, camera transform/name, intake thresholds, and PID gains remain team-provided or placeholder values. The intake constructor still assumes its physical position matches zero at startup. Homing/limits, current limits, inversion, shooter readiness/interpolation, vision calibration/rejection, and autonomous tuning are outside this seven-bug exercise. The existing PathPlanner settings must be present; otherwise the supplied AutoBuilder setup cannot work. There is no promise of a working physical mechanism model in simulation.

## Suggested session

- 5 minutes: explain requirements and the build/debug workflow.
- 10 minutes: individual inspection and first build fix.
- 25 minutes: pairs investigate one behavior at a time.
- 10 minutes: release hints as needed and finish explanations.
- 10 minutes: compare fixes and discuss verification.

## API references

WPILib documents lifecycle cleanup, startEnd callbacks, and subsystem ownership in [Commands](https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html). Its [PID controller documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html) covers measurement updates, continuous input, and output clamping. CTRE documents FieldCentric rotational rate in radians per second in the [FieldCentric API](https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/swerve/SwerveRequest.FieldCentric.html).

## Complete corrected source files

Use the answer-key source overlay in a separate copy of the existing project. Do not compile student and answer-key trees together.

### src/main/java/frc/robot/commands/RotateToHub.java

```java
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
        measuredHeading = robotPose.getRotation().getRadians();
        Rotation2d targetRotation = getTargetRotation(robotPose);
        double rotationSpeed = pidController.calculate(
            measuredHeading, targetRotation.getRadians());
        rotationSpeed = MathUtil.clamp(
            rotationSpeed, -MAX_ROTATION_RATE, MAX_ROTATION_RATE);

        drivetrain.setControl(driveRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(rotationSpeed));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(driveRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public static Translation2d getHubCoordinates() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Red
            ? Constants.RED_HUB_COORDINATES
            : Constants.BLUE_HUB_COORDINATES;
    }

    private static Rotation2d getTargetRotation(Pose2d robotPose) {
        Translation2d toHubVector =
            getHubCoordinates().minus(robotPose.getTranslation());
        return toHubVector.getAngle();
    }
}
```


### src/main/java/frc/robot/commands/ShootFuel.java

```java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;

public class ShootFuel extends Command {
    private final Flywheel flywheel;
    private final Hopper hopper;

    public ShootFuel(Flywheel flywheel, Hopper hopper) {
        this.flywheel = flywheel;
        this.hopper = hopper;
        addRequirements(flywheel, hopper);
    }

    @Override
    public void initialize() {
        flywheel.spinFlywheel(5);
        hopper.setHopperVoltages(-8, -3);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        flywheel.spinFlywheel(0);
        hopper.setHopperVoltages(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
```


### src/main/java/frc/robot/subsystems/Flywheel.java

```java
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
    private final TalonFX leftFlywheelMotor = new TalonFX(21);
    private final TalonFX rightFlywheelMotor = new TalonFX(20);

    public Flywheel() {}

    public Command spinFlywheel() {
        return startEnd(
            () -> setFlywheelVoltage(3),
            () -> setFlywheelVoltage(0));
    }

    public void spinFlywheel(double volts) {
        setFlywheelVoltage(volts);
    }

    private void setFlywheelVoltage(double volts) {
        leftFlywheelMotor.setVoltage(volts);
        rightFlywheelMotor.setVoltage(volts);
    }
}
```


### src/main/java/frc/robot/subsystems/Intake.java

```java
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
        return startEnd(
            () -> extenderMotor.setVoltage(1),
            () -> extenderMotor.setVoltage(0));
    }

    public Command lowerIntake() {
        return startEnd(
            () -> extenderMotor.setVoltage(-1),
            () -> extenderMotor.setVoltage(0));
    }

    public Command fullyRaiseIntake() {
        final double tolerance = 0.03;
        return new FunctionalCommand(
            () -> extenderMotor.setVoltage(1),
            () -> {},
            interrupted -> extenderMotor.setVoltage(0),
            () -> extenderMotor.getPosition().getValueAsDouble()
                >= -0.05 - tolerance,
            this);
    }

    public Command fullyLowerIntake() {
        final double tolerance = 0.01;
        return new FunctionalCommand(
            () -> extenderMotor.setVoltage(-1),
            () -> {},
            interrupted -> extenderMotor.setVoltage(0),
            () -> extenderMotor.getPosition().getValueAsDouble()
                <= -0.28076171875 + tolerance,
            this);
    }

    public Command setExtenderPositionZero() {
        return runOnce(() -> extenderMotor.setPosition(0));
    }

    public Command runScooper() {
        return startEnd(
            () -> scooperMotor.setVoltage(2),
            () -> scooperMotor.setVoltage(0));
    }

    public Command runPusher() {
        return startEnd(
            () -> pusherMotor.setVoltage(2),
            () -> pusherMotor.setVoltage(0));
    }

    public Command intakeFuel() {
        return startEnd(
            () -> setScooperAndPusherVoltages(5),
            () -> setScooperAndPusherVoltages(0));
    }

    private void setScooperAndPusherVoltages(double volts) {
        scooperMotor.setVoltage(volts);
        pusherMotor.setVoltage(volts);
    }

    @Override
    public void periodic() {}
}
```


### src/main/java/frc/robot/Constants/Constants.java

```java
package frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    private Constants() {}

    public static final Translation2d RED_HUB_COORDINATES =
        new Translation2d(11.921, 4.024);
    public static final Translation2d BLUE_HUB_COORDINATES =
        new Translation2d(4.624, 4.024);
}
```


### src/main/java/frc/robot/subsystems/Hopper.java

```java
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
    private final TalonFX kickerMotor = new TalonFX(14);
    private final TalonFX conveyorMotor = new TalonFX(15);

    public Hopper() {}

    public void setHopperVoltages(double kickerVoltage, double conveyorVoltage) {
        kickerMotor.setVoltage(kickerVoltage);
        conveyorMotor.setVoltage(conveyorVoltage);
    }
}
```


### src/main/java/frc/robot/subsystems/PhotonVision.java

```java
package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEV = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEV = VecBuilder.fill(0.5, 0.5, 1);
    public static final AprilTagFieldLayout tagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final Transform3d robotToCam =
        new Transform3d(0.5, 0.0, 0.5, new Rotation3d(0, 0, 0));
    public Optional<EstimatedRobotPose> visionEst = Optional.empty();

    private static final Matrix<N3, N1> MAX_STD_DEV =
        VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    private final EstimateConsumer estConsumer;
    private final PhotonPoseEstimator photonEstimator;
    private final PhotonCamera camera;
    private Matrix<N3, N1> curStdDevs;

    public PhotonVision(EstimateConsumer estConsumer) {
        this.estConsumer = estConsumer;
        camera = new PhotonCamera("Microsoft_LifeCam_HD-3000-Left-USB2-Vert");
        photonEstimator = new PhotonPoseEstimator(tagLayout, robotToCam);
    }

    @Override
    public void periodic() {
        curStdDevs = SINGLE_TAG_STD_DEV;
        for (var result : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }
            visionEst.ifPresent(est -> {
                updateEstimationStdDevs(visionEst, result.getTargets());
                estConsumer.accept(est.estimatedPose.toPose2d(),
                    est.timestampSeconds, curStdDevs);
            });
        }
    }

    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose,
            List<PhotonTrackedTarget> targets) {
        double totalDistance = 0;
        int tagCount = 0;
        for (PhotonTrackedTarget target : targets) {
            Optional<Pose3d> tagPose = tagLayout.getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) {
                continue;
            }
            Translation2d robotPose =
                estimatedPose.get().estimatedPose.toPose2d().getTranslation();
            totalDistance +=
                robotPose.getDistance(tagPose.get().toPose2d().getTranslation());
            tagCount++;
        }
        if (tagCount == 0) {
            curStdDevs = MAX_STD_DEV;
            return;
        }
        double averageDistance = totalDistance / tagCount;
        double multiplier = 1 + Math.pow(averageDistance, 2) / 30.0;
        curStdDevs = (tagCount == 1 ? SINGLE_TAG_STD_DEV : MULTI_TAG_STD_DEV)
            .times(multiplier);
        if (averageDistance < 1.5) {
            curStdDevs = curStdDevs.times(0.25);
        }
    }

    @FunctionalInterface
    public interface EstimateConsumer {
        void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
```


### src/main/java/frc/robot/Main.java

```java
package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
    private Main() {}

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
```


### src/main/java/frc/robot/Robot.java

```java
package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.RotateToHub;
import frc.robot.commands.ShootFuel;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PhotonVision;

public class Robot extends TimedRobot {
    public static final double MAX_LINEAR_SPEED =
        TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_ANGULAR_SPEED = 1.5 * Math.PI;
    private static final double DEADZONE = 0.15;

    private final Intake intake = new Intake();
    private final Flywheel flywheel = new Flywheel();
    private final Hopper hopper = new Hopper();
    private final Field2d robotFieldWidget = new Field2d();
    private final Field2d cameraFieldWidget = new Field2d();
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final PhotonVision vision = new PhotonVision(drivetrain::addVisionMeasurement);
    private final SendableChooser<Command> autoChooser;
    private final SwerveRequest.FieldCentric swerveRequest = new SwerveRequest.FieldCentric();
    private final CommandXboxController controller = new CommandXboxController(0);
    private Command autonomousCommand;

    public Robot() {
        addCommands();
        drivetrain.configureAutoBuilder();
        controllerTesting();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Robot Field", robotFieldWidget);
        SmartDashboard.putData("Camera Field", cameraFieldWidget);
    }

    public void controllerTesting() {
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> swerveRequest
            .withVelocityY(MathUtil.applyDeadband(-controller.getLeftX(), DEADZONE)
                * MAX_LINEAR_SPEED)
            .withVelocityX(MathUtil.applyDeadband(-controller.getLeftY(), DEADZONE)
                * MAX_LINEAR_SPEED)
            .withRotationalRate(MathUtil.applyDeadband(-controller.getRightX(), DEADZONE)
                * MAX_ANGULAR_SPEED)));

        controller.leftBumper().whileTrue(intake.raiseIntake());
        controller.leftTrigger().whileTrue(intake.lowerIntake());
        controller.rightBumper().onTrue(intake.fullyRaiseIntake());
        controller.rightTrigger().whileTrue(new ShootFuel(flywheel, hopper));
        controller.y().toggleOnTrue(intake.intakeFuel());
        controller.x().onTrue(intake.setExtenderPositionZero());
        controller.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        controller.b().whileTrue(new RotateToHub(drivetrain));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotFieldWidget.setRobotPose(drivetrain.getState().Pose);
        vision.visionEst.ifPresent(est ->
            cameraFieldWidget.setRobotPose(est.estimatedPose.toPose2d()));
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = autoChooser.getSelected();
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
            autonomousCommand = null;
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    public Command getAuto1() {
        return new PathPlannerAuto("Auto 1");
    }

    public void addCommands() {
        NamedCommands.registerCommand("IntakeFuel", intake.intakeFuel());
    }
}
```


### src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java

```java
// @formatter:off
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 *
 * Generated by the 2026 Tuner X Swerve Project Generator
 * https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final SwerveRequest.RobotCentric robotCentricApply = new SwerveRequest.RobotCentric();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    /**
     * Return the pose at a given timestamp, if the buffer is not empty.
     *
     * @param timestampSeconds The timestamp of the pose in seconds.
     * @return The pose at the given timestamp (or Optional.empty() if the buffer is empty).
     */
    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Returns the current robot-relative ChassisSpeeds.
     * PathPlanner uses this to determine the robot's true current velocity.
     * 
     * @return The current robot-relative ChassisSpeeds
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return getState().Speeds;
    }

    private final ApplyRobotSpeeds applyRobotSpeeds = new ApplyRobotSpeeds();

    /**
     * Outputs commands to the robot's drive motors given robot-relative ChassisSpeeds.
     * PathPlanner uses this to send autonomous driving steering and speed commands.
     * 
     * @param robotRelativeSpeeds The target robot-relative ChassisSpeeds
     */

    public void driveRobotRelative(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        this.setControl(
            applyRobotSpeeds
                .withSpeeds(ChassisSpeeds.discretize(speeds, 0.02))
                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
        );
    }

    public Pose2d getPose() {
        return getState().Pose;
    }


    public void configureAutoBuilder() {
        RobotConfig config = null;
        try{
        config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
        }
        if (config != null) {
            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getPose, // Robot pose supplier
                    this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> driveRobotRelative(speeds, feedforwards), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                    ),
                    config, // The robot configuration
                    () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                    },
                    this // Reference to this subsystem to set requirements
            );
        }
    }


}
```
