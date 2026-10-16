package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.Constants.Constants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class ShootFuel extends Command {

    Flywheel flywheel;
    Hopper hopper;
    CommandSwerveDrivetrain drivetrain;
    Translation2d hub = Constants.RED_HUB_COORDINATES;
    public double flywheelVelocity;

    public void setHub() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                hub = Constants.RED_HUB_COORDINATES;

            } else if (alliance.get() == Alliance.Blue) {
                hub = Constants.BLUE_HUB_COORDINATES;
            } else System.out.println("No hub Found");
        }
    }

    public double getTargetFlywheelVelocity(CommandSwerveDrivetrain drivetrain) {
        Translation2d robotPosition = drivetrain.getPose().getTranslation();
        double distanceToHub = robotPosition.getDistance(hub);
        return Constants.flywheelSpeeds.get(distanceToHub);
    }


    public ShootFuel(Flywheel flywheel, Hopper hopper, CommandSwerveDrivetrain drivetrain) {
        this.flywheel = flywheel;
        this.hopper = hopper;
        this.drivetrain = drivetrain;
        addRequirements(flywheel, hopper, drivetrain);
    }

    @Override
    public void initialize() {
        /*
         * We need to find the voltage based on the RPM that we get Currently, we divide by 60 to convert. We need to
         * actually convert RPM to voltage successfully.
         */
        double targetVoltage = (getTargetFlywheelVelocity(drivetrain)) / flywheel.dividingConstant; // need to convert RPM to voltage
        System.out.println("Left flywheel voltage: " + flywheel.getleftFlywheelMotorVoltage());
        System.out.println("Right flywheel voltage: " + flywheel.getRightFlywheelMotorVoltage());
        flywheel.spinFlywheel(targetVoltage * flywheel.flywheelMultiplier);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        hopper.setHopperVoltages(-8, -3);

    }

    @Override
    public void execute() {


    }

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
