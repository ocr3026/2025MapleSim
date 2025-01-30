package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.moduleTranslations;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Drive extends SubsystemBase {
	static final Lock odometryLock = new ReentrantLock();
	private final GyroIO gyroIO;
	private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	private final Module[] modules = new Module[4];
	private final SysIdRoutine sysId;
	private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

	private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
	private Rotation2d rawGyroRotation = new Rotation2d();
	private final SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
		new SwerveModulePosition(),
		new SwerveModulePosition(),
		new SwerveModulePosition(),
		new SwerveModulePosition()
	};
	private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
	private final Consumer<Pose2d> resetSimulationPoseCallback;

	public Drive(
		GyroIO gyroIO,
		ModuleIO flModuleIO,
		ModuleIO frModuleIO,
		ModuleIO rlModuleIO,
		ModuleIO rrModuleIO,
		Consumer<Pose2d> resetSimulationPoseCallback
	) {
		this.gyroIO = gyroIO;
		this.resetSimulationPoseCallback = resetSimulationPoseCallback;
		modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(rlModuleIO, 2);
        modules[3] = new Module(rrModuleIO, 3);

		HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

		SparkOdometryThread.getInstance().start();

		AutoBuilder.configure(
			this::getPose
		);
	}


}
