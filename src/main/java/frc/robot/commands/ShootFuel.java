package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.Constants.Constants;
import edu.wpi.first.math.geometry.Translation2d;

public class ShootFuel extends Command {

    Flywheel flywheel;
    Hopper hopper;
    CommandSwerveDrivetrain drivetrain;
    Translation2d hub = Constants.RED_HUB_COORDINATES;

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
