package frc.autonomous;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristSubsystem;

public class FR_2Coral extends AutoBase {

	public FR_2Coral(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		addCommands(delayStartTime());
		addCommands(setStartPose(Paths.FR_C5));
		addCommands(followPath(Paths.FR_C5));
		addCommands(moveElevatorAndOuttake(wrist, elevator, ElevatorPos.LOW));
		addCommands(followPath(Paths.C5_FeedR));
		addCommands(feedCoralCommand(elevator, wrist));
		addCommands(followPath(Paths.FeedR_C5));
		addCommands(moveElevatorAndOuttakeHome(wrist, elevator, ElevatorPos.HOME));
	}
}
