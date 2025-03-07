package frc.autonomous;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristSubsystem;

public class FL_C3_2C extends AutoBase {

	public FL_C3_2C(ElevatorSubsystem elevator, WristSubsystem wrist) {
		addCommands(delayStartTime());
		addCommands(setStartPose(Paths.FL_C3));
		addCommands(followPath(Paths.FL_C3));
		addCommands(moveElevatorAndOuttake(wrist, elevator, ElevatorPos.LOW));
		addCommands(followPath(Paths.C3_FeedL));
		addCommands(feedCoralCommand(elevator, wrist));
		addCommands(followPath(Paths.FeedL_C3));
		addCommands(moveElevatorAndOuttakeHome(wrist, elevator, ElevatorPos.HOME));
	}
}
