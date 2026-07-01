package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;

public class ShootFuel extends Command {

    Flywheel flywheel;
    Hopper hopper;
    CommandSwerveDrivetrain drivetrain;


    public ShootFuel(CommandSwerveDrivetrain drivetrain, Flywheel flywheel, Hopper hopper) {
        this.drivetrain = drivetrain;
        this.flywheel = flywheel;
        this.hopper = hopper;
        addRequirements(drivetrain, flywheel, hopper);
    }

    @Override
    public void initialize() {
        flywheel.spinFlywheel(3);
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
