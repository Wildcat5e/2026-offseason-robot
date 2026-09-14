# FRC debugging workshop — student version

Find and fix **seven intentional bugs**. Make the project compile and restore the behavior below. You may build, inspect code, trace values, use a debugger, and consult documentation. Explain the cause and verification for each fix; do not just make the symptom disappear.

Use the files in `student/src/main/java/frc/robot/` to replace their matching files in a COPY of the team's existing project. Do not put both versions in one source tree. Keep the existing Gradle files, vendordeps, generated TunerConstants, deploy/PathPlanner files, license, and other project assets. This packet is a source overlay, not a standalone project.

This is deliberately faulty code: inspect it and use simulation or controlled software checks. Do not deploy the student version to a powered robot. Existing intake position values are classroom fixtures, not newly calibrated physical limits.

## Expected behavior

- The project builds after the compile defect is repaired (assuming the original project dependencies are installed).
- Holding B turns the stationary robot's forward direction toward its alliance hub. The requested turn rate changes as its measured heading changes and approaches zero at the target. Releasing B ends aiming. Unknown alliance uses blue. This is whole-robot aiming, not a separate turret.
- Holding the right trigger requests 5 V on each flywheel motor, -8 V on the kicker, and -3 V on the conveyor. Releasing it requests 0 V on all four. Shooting reserves both subsystems it controls.
- Y toggles fuel intake on and off. Both scooper and pusher receive 5 V while active and a 0 V request on cancellation. Zero voltage does not promise an instantaneous physical stop.
- From a lowered intake position, right bumper raises the intake toward the upper threshold, then requests 0 V. In the exercise, positive extender voltage increases its position; the completion threshold is -0.08 rotations. At -0.20 it should continue; at -0.08 and -0.06 it should finish.
- Manual intake raise/lower and ordinary joystick driving keep their existing behavior.

## Hardware reference

Use this supplied team wiring map to check configuration; do not guess motor IDs.

| Motor | CAN ID |
|---|---:|
| Left flywheel | 21 |
| Right flywheel | 20 |
| Kicker | 14 |
| Conveyor | 15 |
| Pusher | 16 |
| Extender | 17 |
| Scooper | 18 |

## Workflow

1. Build from the project root using `./gradlew build` on macOS/Linux or `.\gradlew.bat build` in Windows PowerShell. Resolve compiler output first.
2. Trace one expected behavior at a time, including command start, repeated execution, and cancellation.
3. Check numeric values and units, not just variable names.
4. For each fix record: symptom, file/method, cause, change, and evidence.
5. Request one hint level at a time if stuck.

There is no flywheel interpolation or speed-readiness model in this exercise: the supplied shooting behavior is open-loop voltage control. The supplied drivetrain has simulation support, but the intake/flywheel/hopper do not have simulated mechanism physics. Use source traces, manually supplied sensor fixtures, or instructor-provided test doubles for their checks; merely launching simulation will not animate these mechanisms.

## Complete source files

Each block below is a complete file. Paths are relative to the existing project root.

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
        Rotation2d targetRotation = getTargetRotation(robotPose);
        double rotationSpeed = pidController.calculate(
            measuredHeading, targetRotation.getDegrees());
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
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        flywheel.spinFlywheel(5);
        hopper.setHopperVoltage(-8, -3);
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
    private final TalonFX rightFlywheelMotor = new TalonFX(21);

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
                <= -0.05 - tolerance,
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
            () -> {});
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
