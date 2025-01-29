package frc.robot.subsystems.drive;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import frc.robot.util.SparkUtil;

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
