package frc.autonomous;

import frc.autonomous.AutoBase.Paths;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristSubsystem;

public class FL_3Coral extends AutoBase {

	public FL_3Coral(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		addCommands(delayStartTime());
		addCommands(setStartPose(Paths.FL_C4));
		addCommands(followPath(Paths.FL_C4));
		addCommands(moveElevatorAndOuttake(wrist, elevator, ElevatorPos.INTAKE));
		addCommands(followPath(Paths.C4_FeedL));
		addCommands(moveElevatorAndIntake(wrist, elevator, ElevatorPos.INTAKE));
		addCommands(followPath(Paths.FeedL_C5));
		addCommands((moveElevatorAndOuttake(wrist, elevator, ElevatorPos.LOW)));
		addCommands(followPath(Paths.C5_FeedL));
		addCommands(moveElevatorAndIntake(wrist, elevator, ElevatorPos.INTAKE));
		addCommands(followPath(Paths.FeedL_C6));
		addCommands(moveElevatorAndOuttake(wrist, elevator, ElevatorPos.LOW));
	}
}
