package frc.autonomous;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristSubsystem;

public class FL_2Coral extends AutoBase {

	/*
	 * Update paths and elevator heights, this is just a starting position
	 */
	public FL_2Coral(WristSubsystem wrist, ElevatorSubsystem elevator) {
		super(elevator, wrist);
		addCommands(delayStartTime());
		addCommands(setStartPose(Paths.FL_C3));
		addCommands(followPath(Paths.FL_C2));
		addCommands(moveElevatorAndOuttake(wrist, elevator, ElevatorPos.LOW));
		addCommands(followPath(Paths.C2_FeedL));
		addCommands(feedCoralCommand(elevator, wrist));
		addCommands(followPath(Paths.FeedL_C2));
		addCommands(moveElevatorAndOuttakeHome(wrist, elevator, ElevatorPos.HOME));
	}
}
