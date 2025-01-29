package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public interface GyroIO {
	@AutoLog
	public static class GyroIOInputs {
		public boolean connected = false;
		public Rotation2d yawPosition = new Rotation2d();
		public AngularVelocity yawVelocity = RadiansPerSecond.of(0);
		public double[] odometryYawTimestamps = new double[] {};
		public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
	}

	public default void updateInputs(GyroIOInputs inputs) {}
}
