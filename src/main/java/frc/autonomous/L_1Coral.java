package frc.autonomous;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristSubsystem;

public class L_1Coral extends AutoBase {

	public L_1Coral(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		addCommands(delayStartTime());
		addCommands(setStartPose(Paths.L_C1));
		addCommands(followPath(Paths.L_C2));
		addCommands(moveElevatorAndOuttakeHome(wrist, elevator, ElevatorPos.LOW));
	}
}
