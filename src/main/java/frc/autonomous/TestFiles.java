package frc.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class TestFiles extends AutoBase {
	boolean b = false;
	PathPlannerPath path;

	public TestFiles(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		addCommands(Commands.runOnce(() -> Paths.initPaths()));
		if (Paths.paths.get("L to C1.path") != null) {
			addCommands(Commands.runOnce(() -> SmartDashboard.putString("testing auto", "path is not null")));
		} else {
			addCommands(Commands.runOnce(() -> SmartDashboard.putString("testing auto", "path is null")));
			addCommands(Commands.runOnce(() -> {
				for (String key : Paths.paths.keySet()) {
					SmartDashboard.putString("keys in map", key);
				}
				path = Paths.paths.get("L to C1");
			}));
		}

		addCommands(followPath(path));
	}
}
