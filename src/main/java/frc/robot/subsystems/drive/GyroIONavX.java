package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static frc.robot.subsystems.drive.DriveConstants.odometryFrequency;

import java.util.Queue;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIONavX implements GyroIO {
	private final AHRS navX = new AHRS(NavXComType.kMXP_SPI, (byte) odometryFrequency);
	private final Queue<Double> yawPositionQueue;
	private final Queue<Double> yawTimestampQueue;

	public GyroIONavX() {
		yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
		yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(navX::getAngle);
	}

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		inputs.connected = navX.isConnected();
		inputs.yawPosition = Rotation2d.fromDegrees(-navX.getAngle());
		inputs.yawVelocity = DegreesPerSecond.of(-navX.getRawGyroZ());

		inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
		inputs.odometryYawPositions = yawPositionQueue.stream()
			.map((Double value) -> Rotation2d.fromDegrees(-value))
			.toArray(Rotation2d[]::new);
		
		yawTimestampQueue.clear();
		yawPositionQueue.clear();
	}
}
