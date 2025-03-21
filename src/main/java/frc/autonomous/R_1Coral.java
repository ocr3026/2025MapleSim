package frc.autonomous;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristSubsystem;

public class R_1Coral extends AutoBase {

	public R_1Coral(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		addCommands(delayStartTime());
		addCommands(setStartPose(Paths.R_C1));
		addCommands(followPath(Paths.R_C1));
		addCommands(moveElevatorAndOuttakeHomeRight(wrist, elevator, ElevatorPos.INTAKE));
	}
}
