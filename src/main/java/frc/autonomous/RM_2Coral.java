package frc.autonomous;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristSubsystem;

public class RM_2Coral extends AutoBase {

	public RM_2Coral(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		addCommands(delayStartTime());
		addCommands(setStartPose(Paths.RM_C2));
		addCommands(followPath(Paths.RM_C12));
		addCommands(moveElevatorAndOuttake(wrist, elevator, ElevatorPos.MID));
		addCommands(followPath(Paths.C12_FeedR));
		addCommands(moveElevatorAndOuttake(wrist, elevator, ElevatorPos.INTAKE));
		addCommands(followPath(Paths.FeedR_C9));
		addCommands(moveElevatorAndOuttake(wrist, elevator, ElevatorPos.MID));
	}
}
