package frc.autonomous;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristSubsystem;

public class LM_2Coral extends AutoBase {

	public LM_2Coral(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		addCommands(delayStartTime());
		addCommands(setStartPose(Paths.LM_C2));
		addCommands(followPath(Paths.LM_C3));
		addCommands(moveElevatorAndOuttake(wrist, elevator, ElevatorPos.MID));
		addCommands(followPath(Paths.C3_FeedL));
		addCommands(moveElevatorAndOuttake(wrist, elevator, ElevatorPos.INTAKE));
		addCommands(followPath(Paths.FeedL_C7));
		addCommands(moveElevatorAndOuttake(wrist, elevator, ElevatorPos.MID));
	}
}
