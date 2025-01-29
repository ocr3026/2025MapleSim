package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import frc.robot.util.SparkUtil;
import java.util.Queue;
import java.util.function.DoubleSupplier;

public class ModuleIOSpark implements ModuleIO {
	private Rotation2d magnetOffset;

	private final SparkMax driveMotor;
	private final SparkMax turnMotor;
	private final RelativeEncoder driveEncoder;
	private final CANcoder turnCancoder;
	private final RelativeEncoder turnRelativeEncoder;

	private final SparkClosedLoopController driveController;
	private final SimpleMotorFeedforward driveFFController = new SimpleMotorFeedforward(driveKs, driveKv, 0);
	private final SparkClosedLoopController turnController;

	private final Queue<Double> timestampQueue;
	private final Queue<Double> drivePositionQueue;
	private final Queue<Double> turnPositionQueue;

	private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
	private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

	public ModuleIOSpark(int module) {
		if (module < 0 || module > 3) {
			throw new IllegalArgumentException("Invalid Module");
		}

		magnetOffset = switch (module) {
			case 0 -> frontLeftMagnetOffset;
			case 1 -> frontRightMagnetOffset;
			case 2 -> rearLeftMagnetOffset;
			case 3 -> rearRightMagnetOffset;
			default -> new Rotation2d();};

		driveMotor = new SparkMax(
				switch (module) {
					case 0 -> frontLeftDriveID;
					case 1 -> frontRightDriveID;
					case 2 -> rearLeftDriveID;
					case 3 -> rearRightDriveID;
					default -> 0;
				},
				MotorType.kBrushless);

		turnMotor = new SparkMax(
				switch (module) {
					case 0 -> frontLeftTurnID;
					case 1 -> frontRightTurnID;
					case 2 -> rearLeftTurnID;
					case 3 -> rearRightTurnID;
					default -> 0;
				},
				MotorType.kBrushless);

		turnRelativeEncoder = turnMotor.getEncoder();
		driveEncoder = driveMotor.getEncoder();

		turnCancoder = new CANcoder(
				switch (module) {
					case 0 -> frontLeftEncoderID;
					case 1 -> frontRightEncoderID;
					case 2 -> rearLeftEncoderID;
					case 3 -> rearRightEncoderID;
					default -> 0;
				});

		driveController = driveMotor.getClosedLoopController();

		SparkMaxConfig driveConfig = new SparkMaxConfig();
		driveConfig
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(driveMotorCurrentLimit)
				.voltageCompensation(12.0);
		driveConfig
				.encoder
				.positionConversionFactor(driveEncoderPositionFactor)
				.velocityConversionFactor(driveEncoderVelocityFactor)
				.uvwMeasurementPeriod(10)
				.uvwAverageDepth(2);
		driveConfig
				.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.positionWrappingEnabled(true)
				.positionWrappingInputRange(turnPIDMinInput.getRotations(), turnPIDMaxInput.getRotations())
				.pidf(driveKp, 0, driveKd, 0);
		driveConfig
				.signals
				.primaryEncoderPositionAlwaysOn(true)
				.primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
				.primaryEncoderVelocityAlwaysOn(true)
				.primaryEncoderVelocityPeriodMs(20)
				.appliedOutputPeriodMs(20)
				.busVoltagePeriodMs(20)
				.outputCurrentPeriodMs(20);
		SparkUtil.tryUntilOk(
				driveMotor,
				5,
				() -> driveMotor.configure(
						driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

		SparkMaxConfig turnConfig = new SparkMaxConfig();
		turnConfig
				.inverted(turnInverted)
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(turnMotorCurrentLimit)
				.voltageCompensation(12.0);
		turnConfig
				.encoder
				.positionConversionFactor(turnEncoderPositionFactor)
				.velocityConversionFactor(turnEncoderVelocityFactor)
				.uvwMeasurementPeriod(10)
				.uvwAverageDepth(2);
		driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(driveKp, 0, driveKd, 0);
		turnConfig
				.signals
				.primaryEncoderPositionAlwaysOn(true)
				.primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
				.primaryEncoderVelocityAlwaysOn(true)
				.primaryEncoderVelocityPeriodMs(20)
				.appliedOutputPeriodMs(20)
				.busVoltagePeriodMs(20)
				.outputCurrentPeriodMs(20);
		SparkUtil.tryUntilOk(
				turnMotor,
				5,
				() -> turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

		turnController = turnMotor.getClosedLoopController();

		CANcoderConfiguration turnCancoderConfiguration = new CANcoderConfiguration();
		turnCancoderConfiguration.MagnetSensor.MagnetOffset = magnetOffset.getRotations();
		turnCancoderConfiguration.MagnetSensor.SensorDirection = encoderDirection;
		turnCancoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = absoluteSensorDiscontinuityPoint;
		turnCancoder.getConfigurator().apply(turnCancoderConfiguration);

		turnRelativeEncoder.setPosition(turnCancoder.getPosition().getValue().in(Rotations));

		timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
		drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveMotor, driveEncoder::getPosition);
		turnPositionQueue =
				SparkOdometryThread.getInstance().registerSignal(turnMotor, turnRelativeEncoder::getPosition);
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		SparkUtil.sparkStickyFault = false;
		SparkUtil.ifOk(driveMotor, driveEncoder::getPosition, (value) -> inputs.drivePosition = Meters.of(value));
		SparkUtil.ifOk(
				driveMotor, driveEncoder::getVelocity, (value) -> inputs.driveVelocity = MetersPerSecond.of(value));
		SparkUtil.ifOk(
				driveMotor,
				new DoubleSupplier[] {driveMotor::getAppliedOutput, driveMotor::getBusVoltage},
				(values) -> inputs.driveAppliedVolts = values[0] * values[1]);
		SparkUtil.ifOk(driveMotor, driveMotor::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
		inputs.driveConnected = driveConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);

		SparkUtil.sparkStickyFault = false;
		SparkUtil.ifOk(
				turnMotor,
				turnRelativeEncoder::getPosition,
				(value) -> inputs.turnPosition = Rotation2d.fromRotations(value));
		SparkUtil.ifOk(turnMotor, turnRelativeEncoder::getVelocity, (value) -> inputs.turnVelocity = RPM.of(value));
		SparkUtil.ifOk(
				turnMotor,
				new DoubleSupplier[] {turnMotor::getAppliedOutput, turnMotor::getBusVoltage},
				(values) -> inputs.turnAppliedVolts = values[0] * values[1]);
		SparkUtil.ifOk(turnMotor, turnMotor::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
		inputs.turnConnected = turnConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);

		inputs.odometryTimestamps =
				timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
		inputs.odometryDrivePositionsMeters =
				drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
		inputs.odometryTurnPositions = turnPositionQueue.stream()
				.map((Double value) -> Rotation2d.fromRotations(value))
				.toArray(Rotation2d[]::new);
		timestampQueue.clear();
		drivePositionQueue.clear();
		turnPositionQueue.clear();
	}

	@Override
	public void setDriveOpenLoop(double output) {
		driveMotor.setVoltage(output);
	}

	@Override
	public void setTurnOpenLoop(double output) {
		turnMotor.setVoltage(output);
	}

	@Override
	public void setDriveVelocity(LinearVelocity velocity) {
		driveController.setReference(
				velocity.in(MetersPerSecond),
				ControlType.kVelocity,
				ClosedLoopSlot.kSlot0,
				driveFFController.calculate(velocity.in(MetersPerSecond)),
				ArbFFUnits.kVoltage);
	}

	@Override
	public void setTurnPosition(Rotation2d rotation) {
		double setpoint = MathUtil.inputModulus(
				rotation.getRotations(), turnPIDMinInput.getRotations(), turnPIDMaxInput.getRotations());
		turnController.setReference(setpoint, ControlType.kPosition);
	}
}
