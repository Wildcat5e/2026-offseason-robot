package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.Constants.Constants;
import java.util.Optional;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ShootFuel extends Command {

    Flywheel flywheel;
    Hopper hopper;
    CommandSwerveDrivetrain drivetrain;
    Translation2d hub = Constants.BLUE_HUB_COORDINATES;
    Optional<Alliance> alliance==DriverStation.getAlliance();

    if(alliance.isPresent()){
        if (alliance.get() == Alliance.Red){ hub = Constants.RED_HUB_COORDINATES}
        else if (alliance.get() == Alliance.Blue) {hub  Constants.BLUE_HUB_COORDINATES}
        else System.out.println("No hub Found");
    }

    public double getFlywheelVelocity(CommandSwerveDrivetrain drivetrain) {
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
        flywheel.spinFlywheel(getFlywheelVelocity(drivetrain));
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
