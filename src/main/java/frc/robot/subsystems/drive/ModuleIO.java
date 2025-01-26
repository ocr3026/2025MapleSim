package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface ModuleIO {
	@AutoLog
	public static class ModuleIOInputs {
		public boolean driveConnected = false;
		public Angle drivePosition = Radians.of(0);
		public AngularVelocity driveVelocity = RadiansPerSecond.of(0);
		public double driveAppliedVolts = 0;
		public double driveCurrentAmps = 0;
		
		public boolean turnConnected = false;
		public Rotation2d turnPosition = new Rotation2d();
		public AngularVelocity turnVelocity = RadiansPerSecond.of(0);
		public double turnAppliedVolts = 0;
		public double turnCurrentAmps = 0;

		public double[] odometryTimestamps = new double[] {};
		public Angle[] odometryDrivePositions = new Angle[] {};
		public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
	}

	public default void updateInputs(ModuleIOInputs inputs) {}

	public default void setDriveOpenLoop(double output) {}

	public default void setTurnOpenLoop(double output) {}

	public default void setDriveVelocity(double velocityRadPerSec) {}

	public default void setTurnPosition(Rotation2d rotation) {}
}
