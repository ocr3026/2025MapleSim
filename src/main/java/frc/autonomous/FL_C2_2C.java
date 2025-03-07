package frc.autonomous;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristSubsystem;

public class FL_C2_2C extends AutoBase {

	public FL_C2_2C(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		addCommands(delayStartTime());
		addCommands(setStartPose(Paths.FL_C2));
		addCommands(followPath(Paths.FL_C2));
		addCommands(moveElevatorAndOuttake(wrist, elevator, ElevatorPos.LOW));
		addCommands(followPath(Paths.C2_FeedL));
		addCommands(feedCoralCommand(elevator, wrist));
		addCommands(followPath(Paths.FeedL_C2));
		addCommands(moveElevatorAndOuttakeHome(wrist, elevator, ElevatorPos.HOME));
	}
}
