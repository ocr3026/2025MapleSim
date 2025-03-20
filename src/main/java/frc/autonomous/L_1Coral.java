package frc.autonomous;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristSubsystem;

public class L_1Coral extends AutoBase {

	public L_1Coral(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		addCommands(delayStartTime());
		addCommands(setStartPose(Paths.FL_C1));
		addCommands(followPath(Paths.FL_C5));
		addCommands(moveElevatorAndOuttakeHomeRight(wrist, elevator, ElevatorPos.INTAKE));
		addCommands(followPath(Paths.C5_FeedL));
		addCommands(setElevatorSetpoint(ElevatorPos.INTAKE, elevator));
		addCommands(wristIntake(wrist, elevator));
		addCommands(followPath(Paths.FeedL_C6));
		addCommands(moveElevatorAndOuttakeHomeLeft(wrist, elevator, ElevatorPos.INTAKE));
	}
}
