package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.wheelRadius;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Module {
	private final ModuleIO io;
	private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
	private final int index;

	private final Alert driveDisconnectedAlert;
	private final Alert turnDisconnectedAlert;
	private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

	public Module(ModuleIO io, int index) {
		this.io = io;
		this.index = index;
		driveDisconnectedAlert =
				new Alert("Disconnected drive motor on module " + Integer.toString(index) + ".", AlertType.kError);
		turnDisconnectedAlert =
				new Alert("Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
	}

	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

		int sampleCount = inputs.odometryTimestamps.length;
		odometryPositions = new SwerveModulePosition[sampleCount];
		for (int i = 0; i < sampleCount; i++) {
			double positionMeters = inputs.odometryDrivePositionsMeters[i];
			Rotation2d angle = inputs.odometryTurnPositions[i];
			odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
		}

		driveDisconnectedAlert.set(!inputs.driveConnected);
		turnDisconnectedAlert.set(!inputs.turnConnected);
	}

	public void runSetpoint(SwerveModuleState state) {
		state.optimize(getAngle());
		state.cosineScale(inputs.turnPosition);

		io.setDriveVelocity(MetersPerSecond.of(state.speedMetersPerSecond));
		io.setTurnPosition(state.angle);
	}

	public void runCharacterization(double output) {
		io.setDriveOpenLoop(output);
		io.setTurnPosition(new Rotation2d());
	}

	public void stop() {
		io.setDriveOpenLoop(0);
		io.setTurnOpenLoop(0);
	}

	public Rotation2d getAngle() {
		return inputs.turnPosition;
	}

	public Distance getPosition() {
		return inputs.drivePosition;
	}

	public LinearVelocity getVelocity() {
		return inputs.driveVelocity;
	}

	public SwerveModulePosition getModulePosition() {
		return new SwerveModulePosition(getPosition(), getAngle());
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(getVelocity(), getAngle());
	}

	public SwerveModulePosition[] getOdometryPositions() {
		return odometryPositions;
	}

	public double[] getOdometryTimestamps() {
		return inputs.odometryTimestamps;
	}

	public Angle getWheelRadiusCharacterizationPosition() {
		return Radians.of(inputs.drivePosition.in(Meters) / wheelRadius.in(Meters));
	}

	public AngularVelocity getFFCharacterizationVelocity() {
		return RadiansPerSecond.of(inputs.driveVelocity.in(MetersPerSecond) / wheelRadius.in(Meters));
	}
}
