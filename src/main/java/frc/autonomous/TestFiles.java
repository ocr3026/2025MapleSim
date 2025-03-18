package frc.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class TestFiles extends AutoBase {

	public TestFiles(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);

		addCommands(followPath(Paths.paths.get("C1 to FeedL")));
	}
}
