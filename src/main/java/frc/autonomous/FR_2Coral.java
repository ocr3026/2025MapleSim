package frc.autonomous;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristSubsystem;

public class FR_2Coral extends AutoBase {

	public FR_2Coral(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		addCommands(delayStartTime());
		addCommands(setStartPose(Paths.FR_C10));
		addCommands(followPath(Paths.FR_C10));
		addCommands(moveElevatorAndOuttakeHomeRight(wrist, elevator, ElevatorPos.INTAKE));
		addCommands(followPath(Paths.C10_FeedR));
		addCommands(setElevatorSetpoint(ElevatorPos.INTAKE, elevator));
		addCommands(wait(0.5));
		addCommands(feedCoralCommand(elevator, wrist));
		addCommands(followPath(Paths.FeedR_C9));
		addCommands(moveElevatorAndOuttakeHomeRight(wrist, elevator, ElevatorPos.INTAKE));
	}
}
