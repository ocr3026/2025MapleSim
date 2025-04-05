package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;
import static frc.robot.subsystems.drive.DriveConstants.odometryFrequency;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.util.SparkUtil;

public class ClimberIOSpark implements ClimberIO {
	private final SparkMax climbMotor;
	private final RelativeEncoder climbEncoder;
	private final SparkClosedLoopController climbSparkPID;

	private final SparkMax trapdoorMotor;
	private final RelativeEncoder trapdoorEncoder;

	// private double appliedVolts = 0;

	public ClimberIOSpark() {
		climbMotor = new SparkMax(climbMotorID, MotorType.kBrushless);
		climbSparkPID = climbMotor.getClosedLoopController();
		climbEncoder = climbMotor.getEncoder();
		SparkMaxConfig climbConfig = new SparkMaxConfig();
		climbConfig.idleMode(IdleMode.kBrake);
		climbConfig
				.signals
				.primaryEncoderPositionAlwaysOn(true)
				.primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency.in(Hertz)))
				.primaryEncoderVelocityAlwaysOn(true)
				.primaryEncoderVelocityPeriodMs(20)
				.appliedOutputPeriodMs(20)
				.busVoltagePeriodMs(20)
				.outputCurrentPeriodMs(20);
		climbConfig.closedLoop.pidf(kP, 0, kD, 0);
		climbConfig
				.encoder
				.positionConversionFactor(encoderPositionFactor)
				.velocityConversionFactor(encoderVelocityFactor);
		SparkUtil.tryUntilOk(
				climbMotor,
				5,
				() -> climbMotor.configure(
						climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

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
		inputs.winchAppliedVolts = climbMotor.getAppliedOutput();
		inputs.winchConnected = true;
		inputs.winchCurrentAmps = (climbMotor.getOutputCurrent());
		inputs.winchPosition = Degrees.of(climbEncoder.getPosition());
		inputs.winchVelocity = DegreesPerSecond.of(climbEncoder.getVelocity());

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
	public void runTrapdoor(double voltage) {
		// if (trapdoorEncoder.getPosition() > -10) {
		trapdoorMotor.setVoltage(voltage);
		// } else {
		// 	stopTrapdoor();
		// }
	}

	@Override
	public void stopTrapdoor() {
		trapdoorMotor.setVoltage(0);
	}
}
