package frc.autonomous;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class TestFiles extends AutoBase {

	public TestFiles(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		addCommands(setStartPose(Paths.firstPathChooser.get()));
		addCommands(followPath(Paths.firstPathChooser.get()));
	}
}
