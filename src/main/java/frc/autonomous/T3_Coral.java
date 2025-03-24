package frc.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class T3_Coral extends AutoBase {

	public T3_Coral(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		PathPlannerPath path = Paths.firstPathChooser.get();
		addCommands(setStartPose(path));
		addCommands(followPath(path));
		addCommands(moveElevatorAndOuttake(wrist, elevator, Paths.firstElevatorPosChooser.get()));
		addCommands(followPath(getPathToFeed(path)));
		// addCommands(moveElevatorAndIntakeNoRace(wrist, elevator, ElevatorPos.INTAKE));
		addCommands(followPath(Paths.secondPathChooser.get()));
		addCommands(moveElevatorAndOuttake(wrist, elevator, Paths.secondElevatorPosChooser.get()));
		addCommands(followPath(getPathToFeed(Paths.secondPathChooser.get())));
		// addCommands(moveElevatorAndIntakeNoRace(wrist, elevator, ElevatorPos.INTAKE));
		// addCommands(followPath(Paths.thirdPathChooser.get()));
		addCommands(moveElevatorAndOuttake(wrist, elevator, Paths.thirdElevatorPosChooser.get()));
	}
}
