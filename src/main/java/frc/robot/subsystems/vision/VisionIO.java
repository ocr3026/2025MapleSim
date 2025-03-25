package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
	@AutoLog
	class VisionIOInputs {
		public boolean connected = false;
		public TargetObservation latestTargetObservation =
				new TargetObservation(new Rotation2d(), new Rotation2d(), new Rotation3d());
		public PoseObservation[] poseObservations = new PoseObservation[0];
		public int[] tagIds = new int[0];
	}

	record TargetObservation(Rotation2d tx, Rotation2d ty, Rotation3d tOmega) {}

	record PoseObservation(double timestamp, Pose3d pose, double ambiguity, int tagCount, double averageTagDistance) {}

	default void updateInputs(VisionIOInputs inputs) {}
}
