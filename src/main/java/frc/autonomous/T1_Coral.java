package frc.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class T1_Coral extends AutoBase {

	public T1_Coral(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		Pose2d coralPose = getPathToFeed(Paths.firstPathChooser.get())
				.getStartingHolonomicPose()
				.get();
		addCommands(setStartPose(Paths.firstPathChooser.get()));
		addCommands(pathFindToPose(coralPose));
		addCommands(moveElevatorAndOuttake(wrist, elevator, Paths.firstElevatorPosChooser.get()));
	}
}
