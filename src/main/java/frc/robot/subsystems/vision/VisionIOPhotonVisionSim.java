package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagFieldLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
	private static VisionSystemSim visionSim;

	private final Supplier<Pose2d> poseSupplier;
	private final PhotonCameraSim cameraSim;

	public VisionIOPhotonVisionSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
		super(name, robotToCamera);
		this.poseSupplier = poseSupplier;

		if (visionSim == null) {
			visionSim = new VisionSystemSim("main");
			visionSim.addAprilTags(aprilTagFieldLayout);
		}

		SimCameraProperties cameraProperties = new SimCameraProperties();
		cameraSim = new PhotonCameraSim(camera, cameraProperties);
		visionSim.addCamera(cameraSim, robotToCamera);
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		visionSim.update(poseSupplier.get());
		super.updateInputs(inputs);
	}
}
