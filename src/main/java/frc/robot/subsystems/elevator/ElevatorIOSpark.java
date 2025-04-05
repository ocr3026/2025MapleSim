package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

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
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.util.SparkUtil;

public class ElevatorIOSpark implements ElevatorIO {
	private final SparkMax leadMotor;
	private final SparkMax followMotor;
	private final RelativeEncoder leadEncoder;
	private final RelativeEncoder followEncoder;
	private final SparkClosedLoopController sparkPID;
	// private final PIDController pid;
	// private final ElevatorFeedforward ff;

	public ElevatorIOSpark() {
		leadMotor = new SparkMax(leadMotorID, MotorType.kBrushless);
		followMotor = new SparkMax(followMotorID, MotorType.kBrushless);
		sparkPID = leadMotor.getClosedLoopController();
		leadEncoder = leadMotor.getEncoder();
		followEncoder = followMotor.getEncoder();

		// ff = new ElevatorFeedforward(kS, kG, kV);
		// pid = new PIDController(kP, kI, kD);
		SparkMaxConfig followConfig = new SparkMaxConfig();
		SparkMaxConfig leadConfig = new SparkMaxConfig();
		followConfig.follow(leadMotorID).idleMode(IdleMode.kBrake);
		leadConfig.idleMode(IdleMode.kBrake);

		followConfig.inverted(true);
		followConfig.smartCurrentLimit(60);
		leadConfig.smartCurrentLimit(60);

		followConfig
				.signals
				.primaryEncoderPositionAlwaysOn(true)
				.primaryEncoderPositionPeriodMs((int) (1000.0 / Constants.logFrequency.in(Hertz)))
				.primaryEncoderVelocityAlwaysOn(true)
				.primaryEncoderVelocityPeriodMs(20)
				.appliedOutputPeriodMs(20)
				.busVoltagePeriodMs(20)
				.outputCurrentPeriodMs(20);
		followConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(kP, 0, kD, 0);

		leadConfig
				.encoder
				.velocityConversionFactor(encoderVelocityFactor)
				.positionConversionFactor(encoderPositionFactor);
		followConfig
				.encoder
				.positionConversionFactor(encoderPositionFactor)
				.velocityConversionFactor(encoderVelocityFactor);
		leadConfig
				.signals
				.primaryEncoderPositionAlwaysOn(true)
				.primaryEncoderPositionPeriodMs((int) (1000.0 / Constants.logFrequency.in(Hertz)))
				.primaryEncoderVelocityAlwaysOn(true)
				.primaryEncoderVelocityPeriodMs(20)
				.appliedOutputPeriodMs(20)
				.busVoltagePeriodMs(20)
				.outputCurrentPeriodMs(20);
		leadConfig.closedLoop.pidf(kP, kI, kD, 0).maxOutput(0.5).minOutput(-0.25);
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

	/** @param inputs */
	@Override
	public void updateInputs(ElevatorIOInputs inputs) {
		inputs.masterAppliedVolts = leadMotor.getAppliedOutput();
		inputs.masterConnected = true;
		inputs.masterCurrentAmps = leadMotor.getOutputCurrent();
		inputs.masterPosition = Meter.of(leadEncoder.getPosition());
		inputs.masterVelocity = MetersPerSecond.of(leadEncoder.getVelocity());

		inputs.followAppliedVolts = followMotor.getAppliedOutput();
		inputs.followConnected = true;
		inputs.followCurrentAmps = followMotor.getOutputCurrent();
		inputs.followPosition = Meter.of(followEncoder.getPosition());
		inputs.followVelocity = MetersPerSecond.of(followEncoder.getVelocity());
	}

	/** @param position */
	@Override
	public void setPosition(Distance position) {
		sparkPID.setReference(
				Math.min(position.in(Meter), softwareLimit.in(Meters)),
				ControlType.kPosition,
				ClosedLoopSlot.kSlot0,
				0,
				ArbFFUnits.kVoltage);

		// ffValue = ff.calculate(pid.calculate(leadEncoder.getPosition(), Math.min(position.in(Meter),
		// softwareLimit.in(Meter))));

	}

	/** @return Distance */
	@Override
	public Distance getPosition() {
		return Meters.of(leadEncoder.getPosition());
	}

	/**
	 * @param givenPos
	 * @return Distance
	 */
	@Override
	public Distance getTargetPosition(ElevatorPos givenPos) {
		switch (givenPos) {
			case HIGH:
				return ((ElevatorCommands.highPOS));
			case MIDALGAE:
				return ((ElevatorCommands.midAlgaePOS));
			case MID:
				return ((ElevatorCommands.midPOS));
			case LOWALGAE:
				return ((ElevatorCommands.lowAlgaePOS));
			case LOW:
				return ((ElevatorCommands.lowPOS));
			case INTAKE:
				return ((ElevatorCommands.intakePOS));
			default:
				return null;
		}
	}

	
	/** 
	 * @param speed
	 */
	@Override
	public void setSpeed(double speed) {
		if (Meters.of(leadEncoder.getPosition()).in(Inches) <= softwareLimit.in(Inches)) {
			leadMotor.set(speed);
		} else {
			leadMotor.set(MathUtil.clamp(speed, -0.1, 0));
		}
	}

	@Override
	public void tick() {
		// leadMotor.set(MathUtil.clamp(ffValue, 0, 0.4));
		// leadMotor.set(MathUtil.clamp(motorSpeed, -.25, 1));
	}

	@Override
	public void zeroElevator() {
		// TODO Auto-generated method stub
		leadEncoder.setPosition(0);
	}
}
