package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.climber.ClimberConstants.*;
import static frc.robot.subsystems.drive.DriveConstants.odometryFrequency;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.util.SparkUtil;

public class ClimberIOFalcon implements ClimberIO {
	private final TalonFX climbMotor;

	private final SparkMax trapdoorMotor;
	private final RelativeEncoder trapdoorEncoder;

	// private double appliedVolts = 0;

	public ClimberIOFalcon() {
		climbMotor = new TalonFX(climbMotorID);
		// climbEncoder = climbMotor;
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		climbMotor.getConfigurator().apply(config);

		trapdoorMotor = new SparkMax(trapdoorMotorID, MotorType.kBrushless);
		trapdoorEncoder = trapdoorMotor.getEncoder();

		SparkMaxConfig trapdoorConfig = new SparkMaxConfig();
		trapdoorConfig.idleMode(IdleMode.kBrake);
		trapdoorConfig
				.signals
				.primaryEncoderPositionAlwaysOn(true)
				.primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency.in(Hertz)))
				.primaryEncoderVelocityAlwaysOn(true)
				.primaryEncoderVelocityPeriodMs(20)
				.appliedOutputPeriodMs(20)
				.busVoltagePeriodMs(20)
				.outputCurrentPeriodMs(20);
		SparkUtil.tryUntilOk(
				trapdoorMotor,
				5,
				() -> trapdoorMotor.configure(
						trapdoorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
	}

	@Override
	public void updateInputs(ClimberIOInputs inputs) {
		inputs.winchAppliedVolts = climbMotor.getMotorVoltage().getValue().in(Volts);
		inputs.winchConnected = true;
		inputs.winchCurrentAmps = (climbMotor.getSupplyCurrent().getValue().in(Amps));
		inputs.winchPosition = climbMotor.getPosition().getValue();
		inputs.winchVelocity = climbMotor.getVelocity().getValue();

		inputs.trapdoorAppliedVolts = trapdoorMotor.getAppliedOutput();
		inputs.trapdoorConnected = true;
		inputs.trapdoorCurrentAmps = trapdoorMotor.getOutputCurrent();
		inputs.trapdoorPosition = trapdoorEncoder.getPosition();
		inputs.trapdoorVelocity = RPM.of(trapdoorEncoder.getVelocity());
	}

	@Override
	public void setAngularSpeed(double speed) {
		climbMotor.set(speed);
	}

	@Override
	public void runTrapdoor() {
		if (trapdoorEncoder.getPosition() > -20) {
			trapdoorMotor.setVoltage(openTrapdoorVoltage);
		} else {
			stopTrapdoor();
		}
	}

	@Override
	public void stopTrapdoor() {
		trapdoorMotor.setVoltage(0);
	}
}
