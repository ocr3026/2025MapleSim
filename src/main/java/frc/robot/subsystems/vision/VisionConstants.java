package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.*;

public final class VisionConstants {
	public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

	public static String camera0Name = "camera_0";
	public static String camera1Name = "camera_1";
	
	public static Transform3d robotToCamera0 = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
	public static Transform3d robotToCamera1 = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
	
	public static double maxAmbiguity = 0.3;
	public static double maxZError = 0.75;

	public static Distance linearStdDevBasdeline = Meters.of(0.02);
	public static Angle angularStdDevBaseline = Radians.of(0.06);

	public static double[] cameraStdDevFactors = new double[] {
		1.0,
		1.0
	};

	public static double linearStdDevMegatag2Factor = 0.5;
	public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;
}
