package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.algae.AlgaeConstants.algaeMotorID;
import static frc.robot.subsystems.algae.AlgaeConstants.currentLimit;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants;
import frc.robot.util.SparkUtil;
import java.util.function.DoubleSupplier;

public class AlgaeIOSpark implements AlgaeIO {
	SparkMax motor = new SparkMax(algaeMotorID, MotorType.kBrushless);
	RelativeEncoder encoder = motor.getEncoder();
	private final Debouncer motorConnectedDebounce = new Debouncer(0.5);

	public AlgaeIOSpark() {
		SparkMaxConfig config = new SparkMaxConfig();
		config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit);
		config.signals
				.primaryEncoderPositionAlwaysOn(true)
				.primaryEncoderPositionPeriodMs((int) (1000.0 / Constants.logFrequency.in(Hertz)))
				.primaryEncoderVelocityAlwaysOn(true)
				.primaryEncoderVelocityPeriodMs(20)
				.appliedOutputPeriodMs(20)
				.busVoltagePeriodMs(20)
				.outputCurrentPeriodMs(20);
		SparkUtil.tryUntilOk(
				motor,
				5,
				() -> motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
	}

	@Override
	public void updateInputs(AlgaeIOInputs inputs) {
		SparkUtil.sparkStickyFault = false;
		SparkUtil.ifOk(motor, encoder::getVelocity, (value) -> inputs.motorVelocity = RPM.of(value));
		SparkUtil.ifOk(
				motor,
				new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
				(values) -> inputs.motorAppliedVolts = values[0] * values[1]);
		SparkUtil.ifOk(motor, motor::getOutputCurrent, (value) -> inputs.motorCurrentAmps = value);
		inputs.motorConnected = motorConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);
	}

	@Override
	public void runVoltage(double volts) {
		motor.setVoltage(volts);
	}
}
