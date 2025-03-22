package frc.autonomous;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristSubsystem;

public class T2_Coral extends AutoBase {
	public T2_Coral(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		addCommands(setStartPose(Paths.firstPathChooser.get()));
		addCommands(followPath(Paths.firstPathChooser.get()));
		addCommands(moveElevatorAndOuttake(wrist, elevator, Paths.firstElevatorPosChooser.get()));
		addCommands(followPath(getPathToFeed(Paths.firstPathChooser.get())));
		addCommands(moveElevatorAndIntake(wrist, elevator, ElevatorPos.INTAKE));
		addCommands(followPath(Paths.secondPathChooser.get()));
		addCommands(moveElevatorAndOuttake(wrist, elevator, Paths.secondElevatorPosChooser.get()));
	}
}
