package frc.autonomous;

import frc.autonomous.AutoBase.Paths;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class FL_3Coral extends AutoBase {

	public FL_3Coral(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		followPath(Paths.firstPathChooser.get());
	}
}
