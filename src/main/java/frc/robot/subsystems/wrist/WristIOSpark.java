package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.wrist.WristConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.util.SparkUtil;

public class WristIOSpark implements WristIO {
	private final SparkMax leadMotor;
	private final SparkMax followMotor;
	private final RelativeEncoder leadEncoder;
	private final RelativeEncoder followEncoder;

	public WristIOSpark() {
		leadMotor = new SparkMax(leadMotorID, MotorType.kBrushless);
		followMotor = new SparkMax(followMotorID, MotorType.kBrushless);
		leadEncoder = leadMotor.getEncoder();
		followEncoder = followMotor.getEncoder();
		SparkMaxConfig followConfig = new SparkMaxConfig();
		SparkMaxConfig leadConfig = new SparkMaxConfig();
		followConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).follow(leadMotorID, true);
		followConfig
				.signals
				.primaryEncoderPositionAlwaysOn(true)
				.primaryEncoderPositionPeriodMs((int) (1000.0 / Constants.logFrequency.in(Hertz)))
				.primaryEncoderVelocityAlwaysOn(true)
				.primaryEncoderVelocityPeriodMs(20)
				.appliedOutputPeriodMs(20)
				.busVoltagePeriodMs(20)
				.outputCurrentPeriodMs(20);
		leadConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit);
		leadConfig
				.signals
				.primaryEncoderPositionAlwaysOn(true)
				.primaryEncoderPositionPeriodMs((int) (1000.0 / Constants.logFrequency.in(Hertz)))
				.primaryEncoderVelocityAlwaysOn(true)
				.primaryEncoderVelocityPeriodMs(20)
				.appliedOutputPeriodMs(20)
				.busVoltagePeriodMs(20)
				.outputCurrentPeriodMs(20);
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
	public void updateInputs(WristIOInputs inputs) {
		inputs.leadConnected = true;
		inputs.leadAppliedVolts = leadMotor.getAppliedOutput();
		inputs.leadCurrentAmps = leadMotor.getOutputCurrent();
		inputs.leadPosition = Rotations.of(leadEncoder.getPosition());
		inputs.leadVelocity = RotationsPerSecond.of(leadEncoder.getVelocity());

		inputs.followConnected = true;
		inputs.followAppliedVolts = followMotor.getAppliedOutput();
		inputs.followCurrentAmps = followMotor.getOutputCurrent();
		inputs.followPosition = Rotations.of(followEncoder.getPosition());
		inputs.followVelocity = RotationsPerSecond.of(followEncoder.getVelocity() / 60);
	}

	@Override
	public void setVoltage(double voltage) {
		leadMotor.setVoltage(voltage);
	}
}
