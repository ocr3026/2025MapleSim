package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;
import static frc.robot.subsystems.drive.DriveConstants.odometryFrequency;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.SparkUtil;

public class ClimberSparkIO implements ClimberIO {
	private final SparkMax climbMotor;
	private final RelativeEncoder climbEncoder;
	private final SparkClosedLoopController sparkPID;

	private double appliedVolts = 0;

	public ClimberSparkIO() {
		climbMotor = new SparkMax(climbMotorID, MotorType.kBrushless);
		sparkPID = climbMotor.getClosedLoopController();
		climbEncoder = climbMotor.getEncoder();
		SparkMaxConfig climbConfig = new SparkMaxConfig();
		climbConfig.idleMode(IdleMode.kBrake);
		climbConfig
				.signals
				.primaryEncoderPositionAlwaysOn(true)
				.primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
				.primaryEncoderVelocityAlwaysOn(true)
				.primaryEncoderVelocityPeriodMs(20)
				.appliedOutputPeriodMs(20)
				.busVoltagePeriodMs(20)
				.outputCurrentPeriodMs(20);
		climbConfig.closedLoop.pidf(kP, 0, kD, 0);
		SparkUtil.tryUntilOk(
				climbMotor,
				5,
				() -> climbMotor.configure(
						climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
	}

	@Override
	public void updateInputs(ClimberIOInputs inputs) {
		inputs.appliedVolts = climbMotor.getAppliedOutput();
		inputs.connected = true;
		inputs.currentAmps = (climbMotor.getOutputCurrent());
		inputs.rotationPosition = Degrees.of(climbEncoder.getPosition());
		inputs.rotationVelocity = DegreesPerSecond.of(climbEncoder.getVelocity());
	}

	@Override
	public void setAngularPosition(Angle position) {
		sparkPID.setReference(position.in(Degrees), ControlType.kPosition);
	}

	@Override
	public void setAngularSpeed(double speed) {
		// TODO Auto-generated method stub
		climbMotor.set(speed);
	}

	@Override
	public double getPositionInDegrees() {
		// TODO Auto-generated method stub
		return climbEncoder.getPosition() * 360;
	}
}
