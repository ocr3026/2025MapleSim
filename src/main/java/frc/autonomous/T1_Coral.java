package frc.autonomous;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class T1_Coral extends AutoBase {

	public T1_Coral(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		addCommands(setStartPose(Paths.firstPathChooser.get()));
		addCommands(followPath(Paths.firstPathChooser.get()));
	}
}
