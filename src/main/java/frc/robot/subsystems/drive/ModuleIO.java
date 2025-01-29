package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
	@AutoLog
	public static class ModuleIOInputs {
		public boolean driveConnected = false;
		public Distance drivePosition = Meters.of(0.0);
		public LinearVelocity driveVelocity = MetersPerSecond.of(0);
		public double driveAppliedVolts = 0;
		public double driveCurrentAmps = 0;

		public boolean turnConnected = false;
		public Rotation2d turnPosition = new Rotation2d();
		public AngularVelocity turnVelocity = RadiansPerSecond.of(0);
		public double turnAppliedVolts = 0;
		public double turnCurrentAmps = 0;

		public double[] odometryTimestamps = new double[] {};
		public Distance[] odometryDrivePositions = new Distance[] {};
		public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
	}

	public default void updateInputs(ModuleIOInputs inputs) {}

	public default void setDriveOpenLoop(double output) {}

	public default void setTurnOpenLoop(double output) {}

	public default void setDriveVelocity(LinearVelocity velocityRadPerSec) {}

	public default void setTurnPosition(Rotation2d rotation) {}
}
