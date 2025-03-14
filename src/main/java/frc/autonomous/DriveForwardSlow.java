package frc.autonomous;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class DriveForwardSlow extends AutoBase {

	public DriveForwardSlow(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		addCommands(delayStartTime());
		addCommands(setStartPose(Paths.driveForwardSlow));
		addCommands(followPath(Paths.driveForwardSlow));
	}
}
