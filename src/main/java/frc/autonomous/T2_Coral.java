package frc.autonomous;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class T2_Coral extends AutoBase {
	public T2_Coral(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		addCommands(setStartPose(Paths.firstPathChooser.get()));
		addCommands(followPath(Paths.firstPathChooser.get()));
		addCommands(followPath(getPathToFeed(Paths.firstPathChooser.get())));
		addCommands(followPath(Paths.secondPathChooser.get()));
	}
}
