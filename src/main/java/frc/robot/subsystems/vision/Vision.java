package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
	PowerDistribution distribution = new PowerDistribution();
	private final VisionConsumer consumer;
	private final VisionIO[] io;
	private final VisionIOInputsAutoLogged[] inputs;
	private final Alert[] disconnectedAlerts;

	public Vision(VisionConsumer consumer, VisionIO... io) {
		this.consumer = consumer;
		this.io = io;

		this.inputs = new VisionIOInputsAutoLogged[io.length];
		for (int i = 0; i < inputs.length; i++) {
			inputs[i] = new VisionIOInputsAutoLogged();
		}

		this.disconnectedAlerts = new Alert[io.length];
		for (int i = 0; i < inputs.length; i++) {
			disconnectedAlerts[i] =
					new Alert("Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
		}
	}

	/**
	 * @param cameraIndex
	 * @return Rotation2d
	 */
	public Rotation2d getTargetX(int cameraIndex) {
		return inputs[cameraIndex].latestTargetObservation.tx();
	}

	/**
	 * @param cameraIndex
	 * @return Transform3d
	 */
	public Transform3d getTargetTransform(int cameraIndex) {
		return inputs[cameraIndex].latestTargetObservation.t3d();
	}

	@Override
	public void periodic() {
		Logger.recordOutput("Vision/Current", distribution.getCurrent(15));

		for (int i = 0; i < io.length; i++) {
			io[i].updateInputs(inputs[i]);
			Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
		}

		List<Pose3d> allTagPoses = new LinkedList<>();
		List<Pose3d> allRobotPoses = new LinkedList<>();
		List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
		List<Pose3d> allRobotPosesRejected = new LinkedList<>();

		for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
			disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

			List<Pose3d> tagPoses = new LinkedList<>();
			List<Pose3d> robotPoses = new LinkedList<>();
			List<Pose3d> robotPosesAccepted = new LinkedList<>();
			List<Pose3d> robotPosesRejected = new LinkedList<>();

			for (int tagId : inputs[cameraIndex].tagIds) {
				Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(tagId);
				if (tagPose.isPresent()) {
					tagPoses.add(tagPose.get());
				}
			}

			for (PoseObservation observation : inputs[cameraIndex].poseObservations) {
				boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
						|| (observation.tagCount() == 1
								&& observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
						|| Math.abs(observation.pose().getZ()) > maxZError // Must have realistic Z coordinate

						// Must be within the field boundaries
						|| observation.pose().getX() < 0.0
						|| observation.pose().getX() > aprilTagFieldLayout.getFieldLength()
						|| observation.pose().getY() < 0.0
						|| observation.pose().getY() > aprilTagFieldLayout.getFieldWidth();

				robotPoses.add(observation.pose());
				if (rejectPose) {
					robotPosesRejected.add(observation.pose());
				} else {
					robotPosesAccepted.add(observation.pose());
				}

				if (rejectPose) {
					continue;
				}

				double stdDevFactor = Math.pow(observation.averageTagDistance(), 2) / observation.tagCount();
				double linearStdDev = linearStdDevBaseline.in(Meters) * stdDevFactor;
				double angularStdDev = angularStdDevBaseline.in(Radians) * stdDevFactor;
				if (cameraIndex < cameraStdDevFactors.length) {
					linearStdDev *= cameraStdDevFactors[cameraIndex];
					angularStdDev *= cameraStdDevFactors[cameraIndex];
				}

				consumer.accept(
						observation.pose().toPose2d(),
						observation.timestamp(),
						VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
			}

			Logger.recordOutput(
					"Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
					tagPoses.toArray(new Pose3d[tagPoses.size()]));
			Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/tx", getTargetX(cameraIndex));
			Logger.recordOutput(
					"Vision/Camera" + Integer.toString(cameraIndex) + "/t3d", getTargetTransform(cameraIndex));
			Logger.recordOutput(
					"Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
					robotPoses.toArray(new Pose3d[robotPoses.size()]));
			Logger.recordOutput(
					"Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
					robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
			Logger.recordOutput(
					"Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
					robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
			allTagPoses.addAll(tagPoses);
			allRobotPoses.addAll(robotPoses);
			allRobotPosesAccepted.addAll(robotPosesAccepted);
			allRobotPosesRejected.addAll(robotPosesRejected);
		}

		Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
		Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
		Logger.recordOutput(
				"Vision/Summary/RobotPosesAccepted",
				allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
		Logger.recordOutput(
				"Vision/Summary/RobotPosesRejected",
				allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
	}

	@FunctionalInterface
	public interface VisionConsumer {
		void accept(Pose2d visionRobotMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
	}
}
