package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase implements Vision.VisionConsumer {
	static final Lock odometryLock = new ReentrantLock();
	private final GyroIO gyroIO;
	private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	private final Module[] modules = new Module[4];
	private final SysIdRoutine sysId;
	private final Alert gyroDisconnectedAlert =
			new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

	private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
	private Rotation2d rawGyroRotation = new Rotation2d();
	private final SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
		new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
	};
	private final SwerveDrivePoseEstimator poseEstimator =
			new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
	private final Consumer<Pose2d> resetSimulationPoseCallback;

	private final Field2d field = new Field2d();

	public Drive(
			GyroIO gyroIO,
			ModuleIO flModuleIO, // RED
			ModuleIO frModuleIO, // BLUE
			ModuleIO rlModuleIO, // GREEN
			ModuleIO rrModuleIO, // YELLOW
			Consumer<Pose2d> resetSimulationPoseCallback) {
		this.gyroIO = gyroIO;
		this.resetSimulationPoseCallback = resetSimulationPoseCallback;
		modules[0] = new Module(flModuleIO, 0);
		modules[1] = new Module(frModuleIO, 1);
		modules[2] = new Module(rlModuleIO, 2);
		modules[3] = new Module(rrModuleIO, 3);

		HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

		SparkOdometryThread.getInstance().start();

		AutoBuilder.configure(
				this::getPose,
				this::resetOdometry,
				this::getChassisSpeeds,
				this::runVelocity,
				new PPHolonomicDriveController(new PIDConstants(10, 0, 0), new PIDConstants(5, 0, 0)),
				ppConfig,
				() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
				this);
		Pathfinding.setPathfinder(new LocalADStarAK());
		PathPlannerLogging.setLogActivePathCallback((activePath) -> {
			Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
		});
		PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
			Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
		});

		sysId = new SysIdRoutine(
				new SysIdRoutine.Config(
						null, null, null, (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
				new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Robotx", (getPose()).getX());
		SmartDashboard.putNumber("Roboty", (getPose()).getY());
		SmartDashboard.putNumber("Robotr", (getPose()).getRotation().getDegrees());
		odometryLock.lock();
		gyroIO.updateInputs(gyroInputs);
		Logger.processInputs("Drive/Gyro", gyroInputs);
		for (Module module : modules) {
			module.periodic();
		}
		odometryLock.unlock();

		if (DriverStation.isDisabled()) {
			for (Module module : modules) {
				module.stop();
			}
		}

		if (DriverStation.isDisabled()) {
			Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
			Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
		}

		double[] sampleTimestamps = modules[0].getOdometryTimestamps();
		int sampleCount = sampleTimestamps.length;
		for (int i = 0; i < sampleCount; i++) {
			SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
			SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
			for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
				modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
				moduleDeltas[moduleIndex] = new SwerveModulePosition(
						modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
						modulePositions[moduleIndex].angle);
				lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
			}

			if (gyroInputs.connected) {
				rawGyroRotation = gyroInputs.odometryYawPositions[i];
			} else {
				Twist2d twist = kinematics.toTwist2d(moduleDeltas);
				rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
			}

			field.setRobotPose(poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions));
			SmartDashboard.putData("field", field);
		}

		gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

		// ElevatorSubsystem.mechRoot.setPosition(getPose().getY() - 3, getPose().getX() - 3);
	}

	public void runVelocity(ChassisSpeeds speeds) {
		speeds = ChassisSpeeds.discretize(speeds, 0.02);
		SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeed.in(MetersPerSecond));

		Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
		Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

		for (int i = 0; i < 4; i++) {
			modules[i].runSetpoint(setpointStates[i]);
		}

		Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
	}

	public void runCharacterization(double output) {
		for (Module module : modules) {
			module.runCharacterization(output);
		}
	}

	public void stop() {
		runVelocity(new ChassisSpeeds());
	}

	public void stopWithX() {
		Rotation2d[] headings = new Rotation2d[4];
		for (int i = 0; i < 4; i++) {
			headings[i] = moduleTranslations[i].getAngle();
		}

		kinematics.resetHeadings(headings);
		stop();
	}

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
	}

	@AutoLogOutput(key = "SwerveStates/Measured")
	private SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (int i = 0; i < 4; i++) {
			states[i] = modules[i].getState();
		}
		return states;
	}

	private SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] states = new SwerveModulePosition[4];
		for (int i = 0; i < 4; i++) {
			states[i] = modules[i].getModulePosition();
		}
		return states;
	}

	@AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
	private ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(getModuleStates());
	}

	public Angle[] getWheelRadiusCharacterizationPositions() {
		Angle[] values = new Angle[4];
		for (int i = 0; i < 4; i++) {
			values[i] = modules[i].getWheelRadiusCharacterizationPosition();
		}
		return values;
	}

	public AngularVelocity getFFCharacterizationVelocity() {
		AngularVelocity output = RadiansPerSecond.of(0);
		for (int i = 0; i < 4; i++) {
			output = output.plus(modules[i].getFFCharacterizationVelocity().div(4));
		}
		return output;
	}

	@AutoLogOutput(key = "Odometry/Robot")
	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public Rotation2d getRotation() {
		return getPose().getRotation();
	}

	public void resetOdometry(Pose2d pose) {
		resetSimulationPoseCallback.accept(pose);
		poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
	}

	@Override
	public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
		poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
	}

	public LinearVelocity getMaxLinearSpeed() {
		return maxSpeed;
	}

	public AngularVelocity getMaxAngularSpeed() {
		return RadiansPerSecond.of(maxSpeed.in(MetersPerSecond) / driveBaseRadius.in(Meters) / 4);
	}
}
