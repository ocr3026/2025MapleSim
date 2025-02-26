package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.odometryFrequency;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.SparkUtil;

public class ElevatorSparkIO implements ElevatorIO {
	private final SparkMax leadMotor;
	private final SparkMax followMotor;
	private final RelativeEncoder leadEncoder;
	private final SparkClosedLoopController sparkPID;
	double motorSpeed = 0;

	public ElevatorSparkIO() {
		leadMotor = new SparkMax(leadMotorID, MotorType.kBrushless);
		followMotor = new SparkMax(followMotorID, MotorType.kBrushless);
		sparkPID = leadMotor.getClosedLoopController();
		leadEncoder = leadMotor.getEncoder();
		SparkMaxConfig followConfig = new SparkMaxConfig();
		SparkMaxConfig leadConfig = new SparkMaxConfig();
		followConfig.follow(leadMotorID).idleMode(IdleMode.kBrake);
		followConfig.inverted(true);
		followConfig.smartCurrentLimit(80);
		leadConfig.smartCurrentLimit(80);

		followConfig
				.signals
				.primaryEncoderPositionAlwaysOn(true)
				.primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
				.primaryEncoderVelocityAlwaysOn(true)
				.primaryEncoderVelocityPeriodMs(20)
				.appliedOutputPeriodMs(20)
				.busVoltagePeriodMs(20)
				.outputCurrentPeriodMs(20);

		leadConfig.idleMode(IdleMode.kBrake);
		leadConfig
				.signals
				.primaryEncoderPositionAlwaysOn(true)
				.primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
				.primaryEncoderVelocityAlwaysOn(true)
				.primaryEncoderVelocityPeriodMs(20)
				.appliedOutputPeriodMs(20)
				.busVoltagePeriodMs(20)
				.outputCurrentPeriodMs(20);
		leadConfig.closedLoop.pidf(kP, 0, kD, 0);
		SparkUtil.tryUntilOk(
				followMotor,
				5,
				() -> followMotor.configure(
						followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
		SparkUtil.tryUntilOk(
				leadMotor,
				5,
				() -> leadMotor.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
	}

	@Override
	public void updateInputs(ElevatorIOInputs inputs) {
		inputs.masterAppliedVolts = leadMotor.getAppliedOutput();
		inputs.masterConnected = true;
		inputs.masterCurrentAmps = (leadMotor.getOutputCurrent() + followMotor.getOutputCurrent());
		inputs.masterPosition = Meter.of(leadEncoder.getPosition());
		inputs.masterVelocity = MetersPerSecond.of(leadEncoder.getVelocity());
	}

	@Override
	public void setPosition(Distance position) {
		sparkPID.setReference(position.in(Meter), ControlType.kPosition);
	}

	@Override
	public Distance getPosition() {
		return Meters.of(leadEncoder.getPosition());
	}

	@Override
	public void setSpeed(double speed) {
		motorSpeed = speed;
	}

	@Override
	public void tick() {
		leadMotor.setVoltage(motorSpeed);
	}
}
