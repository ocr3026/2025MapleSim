package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.autonomous.AutoBase.Paths;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
	private static final double DEADBAND = 0.1;
	private static final double ANGLE_KP = 5.0;
	private static final double ANGLE_KD = 0.4;
	private static final AngularVelocity ANGLE_MAX_VELOCITY = RadiansPerSecond.of(8.0);
	private static final AngularAcceleration ANGLE_MAX_ACCELERATION = RadiansPerSecondPerSecond.of(20.0);
	private static final Time FF_START_DELAY = Seconds.of(2.0);
	private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
	private static final AngularVelocity WHEEL_RADIUS_MAX_VELOCITY = RadiansPerSecond.of(0.25);
	private static final AngularAcceleration WHEEL_RADIUS_RAMP_RATE = RadiansPerSecondPerSecond.of(0.05);

	private static final PIDController xPid = new PIDController(4, 0, 0),
			yPid = new PIDController(4, 0, 0),
			omegaPid = new PIDController(6, 0, 0);

	private DriveCommands() {}

	/**
	 * @param x
	 * @param y
	 * @return Translation2d
	 */
	private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
		double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
		Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

		linearMagnitude = linearMagnitude * linearMagnitude;

		return new Pose2d(new Translation2d(), linearDirection)
				.transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
				.getTranslation();
	}

	public static Command joystickDrive(
			Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
		return Commands.run(
				() -> {
					Translation2d linearVelocity =
							getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

					// Apply rotation deadband
					double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

					// Square rotation value for more precise control
					omega = Math.copySign(omega * omega, omega);

					// Convert to field relative speeds & send command
					ChassisSpeeds speeds = new ChassisSpeeds(
							linearVelocity.getX() * drive.getMaxLinearSpeed().in(MetersPerSecond),
							linearVelocity.getY() * drive.getMaxLinearSpeed().in(MetersPerSecond),
							omega * drive.getMaxAngularSpeed().in(RadiansPerSecond));
					boolean isFlipped = DriverStation.getAlliance().isPresent()
							&& DriverStation.getAlliance().get() == Alliance.Red;
					speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
							speeds,
							isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());
					drive.runVelocity(speeds);
				},
				drive);
	}

	public static Command joystickDriveAtAngle(
			Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> rotationSupplier) {

		// Create PID controller
		ProfiledPIDController angleController = new ProfiledPIDController(
				ANGLE_KP,
				0.0,
				ANGLE_KD,
				new TrapezoidProfile.Constraints(
						ANGLE_MAX_VELOCITY.in(RadiansPerSecond), ANGLE_MAX_ACCELERATION.in(RadiansPerSecondPerSecond)));
		angleController.enableContinuousInput(-Math.PI, Math.PI);

		// Construct command
		return Commands.run(
						() -> {
							// Get linear velocity
							Translation2d linearVelocity =
									getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

							// Calculate angular speed
							double omega = angleController.calculate(
									drive.getRotation().getRadians(),
									rotationSupplier.get().getRadians());

							// Convert to field relative speeds & send command
							ChassisSpeeds speeds = new ChassisSpeeds(
									linearVelocity.getX()
											* drive.getMaxLinearSpeed().in(MetersPerSecond),
									linearVelocity.getY()
											* drive.getMaxLinearSpeed().in(MetersPerSecond),
									omega);
							boolean isFlipped = DriverStation.getAlliance().isPresent()
									&& DriverStation.getAlliance().get() == Alliance.Red;
							speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
									speeds,
									isFlipped
											? drive.getRotation().plus(new Rotation2d(Math.PI))
											: drive.getRotation());
							drive.runVelocity(speeds);
						},
						drive)

				// Reset PID controller when command starts
				.beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
	}

	public static Command feedforwardCharacterization(Drive drive) {
		List<Double> velocitySamples = new LinkedList<>();
		List<Double> voltageSamples = new LinkedList<>();
		Timer timer = new Timer();

		return Commands.sequence(
				// Reset data
				Commands.runOnce(() -> {
					velocitySamples.clear();
					voltageSamples.clear();
				}),

				// Allow modules to orient
				Commands.run(
								() -> {
									drive.runCharacterization(0.0);
								},
								drive)
						.withTimeout(FF_START_DELAY),

				// Start timer
				Commands.runOnce(timer::restart),

				// Accelerate and gather data
				Commands.run(
								() -> {
									double voltage = timer.get() * FF_RAMP_RATE;
									drive.runCharacterization(voltage);
									velocitySamples.add(drive.getFFCharacterizationVelocity()
											.in(RadiansPerSecond));
									voltageSamples.add(voltage);
								},
								drive)

						// When cancelled, calculate and print results
						.finallyDo(() -> {
							int n = velocitySamples.size();
							double sumX = 0.0;
							double sumY = 0.0;
							double sumXY = 0.0;
							double sumX2 = 0.0;
							for (int i = 0; i < n; i++) {
								sumX += velocitySamples.get(i);
								sumY += voltageSamples.get(i);
								sumXY += velocitySamples.get(i) * voltageSamples.get(i);
								sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
							}
							double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
							double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

							NumberFormat formatter = new DecimalFormat("#0.00000");
							System.out.println("********** Drive FF Characterization Results **********");
							System.out.println("\tkS: " + formatter.format(kS));
							System.out.println("\tkV: " + formatter.format(kV));
						}));
	}

	public static Command wheelRadiusCharacterization(Drive drive) {
		SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE.in(RadiansPerSecondPerSecond));
		WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

		return Commands.parallel(
				// Drive control sequence
				Commands.sequence(
						// Reset acceleration limiter
						Commands.runOnce(() -> {
							limiter.reset(0.0);
						}),

						// Turn in place, accelerating up to full speed
						Commands.run(
								() -> {
									double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY.in(RadiansPerSecond));
									drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
								},
								drive)),

				// Measurement sequence
				Commands.sequence(
						// Wait for modules to fully orient before starting measurement
						Commands.waitSeconds(1.0),

						// Record starting measurement
						Commands.runOnce(() -> {
							state.positions = Arrays.stream(drive.getWheelRadiusCharacterizationPositions())
									.mapToDouble((Angle angle) -> angle.in(Radians))
									.toArray();
							state.lastAngle = drive.getRotation();
							state.gyroDelta = 0.0;
						}),

						// Update gyro delta
						Commands.run(() -> {
									var rotation = drive.getRotation();
									state.gyroDelta += Math.abs(
											rotation.minus(state.lastAngle).getRadians());
									state.lastAngle = rotation;
								})

								// When cancelled, calculate and print results
								.finallyDo(() -> {
									double[] positions = Arrays.stream(drive.getWheelRadiusCharacterizationPositions())
											.mapToDouble((Angle angle) -> angle.in(Radians))
											.toArray();
									double wheelDelta = 0.0;
									for (int i = 0; i < 4; i++) {
										wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
									}
									double wheelRadius =
											(state.gyroDelta * DriveConstants.driveBaseRadius.in(Meters)) / wheelDelta;

									NumberFormat formatter = new DecimalFormat("#0.000");
									System.out.println("********** Wheel Radius Characterization Results **********");
									System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
									System.out.println(
											"\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
									System.out.println("\tWheel Radius: "
											+ formatter.format(wheelRadius)
											+ " meters, "
											+ formatter.format(Units.metersToInches(wheelRadius))
											+ " inches");
								})));
	}

	public static Pose2d findBestPoseRight(Drive drive) {
		Pose2d bestPose2d;
		if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
			bestPose2d = FlippingUtil.flipFieldPose(
					FlippingUtil.flipFieldPose(drive.getPose()).nearest(Paths.coralPosesRight));
		} else {
			bestPose2d = drive.getPose().nearest(Paths.coralPosesRight);
		}

		Logger.recordOutput("pose2d best", bestPose2d);

		// SmartDashboard.putString(
		// 		"best pose",
		// 		Paths.coralPosesRightMap.get(
		// 				(drive.getPose().nearest(new ArrayList<Pose2d>(Paths.coralPosesRightMap.keySet())))));

		return bestPose2d;
	}

	public static Pose2d findBestPoseLeft(Drive drive) {
		Pose2d bestPose2d;
		if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
			bestPose2d = FlippingUtil.flipFieldPose(
					FlippingUtil.flipFieldPose(drive.getPose()).nearest(Paths.coralPosesLeft));
		} else {
			bestPose2d = drive.getPose().nearest(Paths.coralPosesLeft);
		}

		Logger.recordOutput("pose2d best", bestPose2d);
		// SmartDashboard.putString(
		// 		"best pose",
		// 		Paths.coralPosesLeftMap.get(
		// 				(drive.getPose().nearest(new ArrayList<Pose2d>(Paths.coralPosesLeftMap.keySet())))));

		// for (Pose2d p : Paths.coralPosesLeft) {
		// 	Logger.recordOutput("pose in list", p);
		// }

		return bestPose2d;
	}

	public static Pose2d findBestPoseAlgae(Drive drive) {
		Pose2d bestPose2d;
		if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
			bestPose2d = FlippingUtil.flipFieldPose(
					FlippingUtil.flipFieldPose(drive.getPose()).nearest(Paths.coralPosesAlgae));
		} else {
			bestPose2d = drive.getPose().nearest(Paths.coralPosesAlgae);
		}

		Logger.recordOutput("pose2d best", bestPose2d);
		// SmartDashboard.putString(
		// 		"best pose",
		// 		Paths.coralPosesAlgaeMap.get(
		// 				(drive.getPose().nearest(new ArrayList<Pose2d>(Paths.coralPosesAlgaeMap.keySet())))));

		for (Pose2d p : Paths.coralPosesAlgae) {
			Logger.recordOutput("pose in list", p);
		}

		return bestPose2d;
	}

	public static Command pathfindToPoseRight(Drive drive) {
		omegaPid.enableContinuousInput(-Math.PI, Math.PI);
		return Commands.runEnd(
				() -> {
					Pose2d pose = findBestPoseRight(drive);
					ChassisSpeeds speeds = new ChassisSpeeds(
							-xPid.calculate(pose.getX(), drive.getPose().getX()),
							-yPid.calculate(pose.getY(), drive.getPose().getY()),
							-omegaPid.calculate(
									pose.getRotation().getRadians(),
									drive.getPose().getRotation().getRadians()));
					speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation());
					drive.runVelocity(speeds);
				},
				() -> {
					drive.runVelocity(new ChassisSpeeds());
				},
				drive);
	}

	public static Command pathfindToPoseLeft(Drive drive) {
		omegaPid.enableContinuousInput(-Math.PI, Math.PI);
		return Commands.runEnd(
				() -> {
					Pose2d pose = findBestPoseLeft(drive);
					ChassisSpeeds speeds = new ChassisSpeeds(
							-xPid.calculate(pose.getX(), drive.getPose().getX()),
							-yPid.calculate(pose.getY(), drive.getPose().getY()),
							-omegaPid.calculate(
									pose.getRotation().getRadians(),
									drive.getPose().getRotation().getRadians()));
					speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation());
					drive.runVelocity(speeds);
				},
				() -> {
					drive.runVelocity(new ChassisSpeeds());
				},
				drive);
	}

	public static Command pathfindToPoseAlgae(Drive drive) {
		omegaPid.enableContinuousInput(-Math.PI, Math.PI);
		return Commands.runEnd(
				() -> {
					Pose2d pose = findBestPoseAlgae(drive);
					ChassisSpeeds speeds = new ChassisSpeeds(
							-xPid.calculate(pose.getX(), drive.getPose().getX()),
							-yPid.calculate(pose.getY(), drive.getPose().getY()),
							-omegaPid.calculate(
									pose.getRotation().getRadians(),
									drive.getPose().getRotation().getRadians()));
					speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation());
					drive.runVelocity(speeds);
				},
				() -> {
					drive.runVelocity(new ChassisSpeeds());
				},
				drive);
	}

	private static class WheelRadiusCharacterizationState {
		double[] positions = new double[4];
		Rotation2d lastAngle = new Rotation2d();
		double gyroDelta = 0.0;
	}
}
