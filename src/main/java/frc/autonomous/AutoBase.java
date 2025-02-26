package frc.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public abstract class AutoBase {

	public abstract void init();

	public static PathPlannerPath getPathFromFile(String name) {
		try {
			PathPlannerPath path = PathPlannerPath.fromPathFile(name);
			return path;
		} catch (Exception e) {
			DriverStation.reportError("Cant Find Path : " + e.getMessage(), e.getStackTrace());
			return null;
		}
	}

	public static final class Paths {
		public static final PathPlannerPath TEST_PATH = getPathFromFile("Test 1");
		public static final PathPlannerPath TEST_PATH_2 = getPathFromFile("Test 2");
	}

	public static final Command wait(double time) {
		return new WaitCommand(time);
	}
	;

	public static final Command setStartPose(PathPlannerPath path) {
		if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
			return AutoBuilder.resetOdom(FlippingUtil.flipFieldPose(path.getStartingDifferentialPose()));
		}
		return AutoBuilder.resetOdom(path.getStartingDifferentialPose());
	}
}
