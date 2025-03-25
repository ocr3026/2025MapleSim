package frc.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristSubsystem;

public class T3_Coral extends AutoBase {

	public T3_Coral(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		PathPlannerPath firstPath = Paths.firstPathChooser.get();
		PathPlannerPath secondPath = Paths.secondPathChooser.get();
		PathPlannerPath thirdPath = Paths.thirdPathChooser.get();
		addCommands(setStartPose(firstPath));
		addCommands(followPath(firstPath));
		addCommands(moveElevatorAndOuttake(wrist, elevator, Paths.firstElevatorPosChooser.get()));
		addCommands(followPath(getPathToFeed(firstPath)));
		addCommands(moveElevatorAndIntakeNoRace(wrist, elevator, ElevatorPos.INTAKE));
		addCommands(followPath(secondPath));
		addCommands(moveElevatorAndOuttake(wrist, elevator, Paths.secondElevatorPosChooser.get()));
		addCommands(followPath(getPathToFeed(secondPath)));
		addCommands(moveElevatorAndIntakeNoRace(wrist, elevator, ElevatorPos.INTAKE));
		addCommands(followPath(thirdPath));
		addCommands(moveElevatorAndOuttake(wrist, elevator, Paths.thirdElevatorPosChooser.get()));
	}
}
