package frc.robot.subsystems.drive;

import frc.robot.util.SparkUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
	private final GyroSimulation gyroSimulation;

	public GyroIOSim(GyroSimulation gyroSimulation) {
		this.gyroSimulation = gyroSimulation;
	}

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		inputs.connected = true;
		inputs.yawPosition = gyroSimulation.getGyroReading();
		inputs.yawVelocity = gyroSimulation.getMeasuredAngularVelocity();

		inputs.odometryYawTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
		inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
	}
}
