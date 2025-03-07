package frc.autonomous;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristSubsystem;

public class FR_C5_2C extends AutoBase {

	public FR_C5_2C(ElevatorSubsystem elevator, WristSubsystem wrist) {
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
