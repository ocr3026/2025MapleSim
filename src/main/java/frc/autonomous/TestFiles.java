package frc.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import org.json.simple.parser.ParseException;

public class TestFiles extends AutoBase {
	boolean b = false;
	File filePath = new File("./src/main/deploy/pathplanner/paths");
	File[] files = filePath.listFiles();
	HashMap<String, PathPlannerPath> paths = new HashMap<>();

	public TestFiles(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		addCommands(Commands.runOnce(() -> {
			for (File f : files) {
				SmartDashboard.putString("filePath", f.getName());
				try {
					paths.put(f.getName(), PathPlannerPath.fromPathFile(f.getName()));
				} catch (FileVersionException | IOException | ParseException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}));
		addCommands(setStartPose(paths.get("C1 to FeedL.path")));
		addCommands(followPath(paths.get("C1 to FeedL.path")));
	}
}
