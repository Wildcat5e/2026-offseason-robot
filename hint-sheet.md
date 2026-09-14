# FRC debugging workshop — progressive hint sheet

Release one hint level at a time. These are the same seven defects in the student source; this sheet is not another set of Java files.

| Bug | Hint 1: behavior | Hint 2: location | Hint 3: specific direction |
|---|---|---|---|
| 1 | Start with what prevents a build. | Compare ShootFuel.initialize() with the Hopper public API. | Check the exact spelling of the method used to start the hopper, including its final letter. |
| 2 | Automatic raising can finish while the intake is still lowered. | Inspect Intake.fullyRaiseIntake()'s completion supplier. | Substitute -0.20, -0.08, and -0.06 into the comparison. Position increases while raising. Which inequality gives false, true, true? |
| 3 | Aiming uses an incorrect angle scale. | Inspect RotateToHub.execute()'s PID arguments and constructor configuration. | Compare the units returned by the target-angle getter with the measurement, tolerance, and continuous-input range. |
| 4 | Even with matching angle units, turning may not respond to the changing heading. | Trace measuredHeading from initialize() through successive execute() calls. | Reading a fresh Pose2d does not automatically update a separate stored double. Does the PID receive the current pose's rotation each cycle? |
| 5 | Another hopper command would be allowed to run during shooting. | Inspect ShootFuel's constructor. | Compare every subsystem receiving motor commands with every subsystem passed to addRequirements(). |
| 6 | Canceling the intake command fails to request zero output. | Inspect Intake.intakeFuel(). | startEnd has one callback for starting and another for ending. What does the second callback do for both motors? |
| 7 | The flywheel objects do not address the two intended devices. | Compare Flywheel's TalonFX constructors with the handout's wiring map. | Each constructor's integer identifies a CAN device. Are both objects addressing the same ID? |

Aiming has two independent defects in the same PID calculation. Fix the unit mismatch and stale measurement separately, checking intermediate values; repairing only one is not expected to restore aiming.
