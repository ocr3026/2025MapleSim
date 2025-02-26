package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;

public class ClimberIOFalcon implements ClimberIO {
	private final TalonFX climbMotor;
	private final Slot0Configs TalonPID = new Slot0Configs();
	private final PositionVoltage positionVoltage;

	private double appliedVolts = 0;

	public ClimberIOFalcon() {
		climbMotor = new TalonFX(climbMotorID);
		TalonPID.kP = 0;
		TalonPID.kI = 0;
		TalonPID.kD = 0;
		positionVoltage = new PositionVoltage(0).withSlot(0);
		// climbEncoder = climbMotor;
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = 40;
		config.CurrentLimits.SupplyCurrentLowerLimit = 30;
		config.CurrentLimits.SupplyCurrentLowerTime = 1;
		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		config.ClosedLoopGeneral.ContinuousWrap = true;
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		climbMotor.getConfigurator().apply(config);
		climbMotor.getConfigurator().apply(TalonPID);
	}

	@Override
	public void updateInputs(ClimberIOInputs inputs) {
		inputs.appliedVolts = climbMotor.getMotorVoltage().getValue().in(Volts);
		inputs.connected = true;
		inputs.currentAmps = (climbMotor.getSupplyCurrent().getValue().in(Amps));
		inputs.rotationPosition = climbMotor.getPosition().getValue();
		inputs.rotationVelocity = climbMotor.getVelocity().getValue();
	}

	@Override
	public void setAngularPosition(Angle position) {
		climbMotor.setControl(positionVoltage.withPosition(position));
	}

	@Override
	public void setAngularSpeed(double speed) {
		// TODO Auto-generated method stub
		climbMotor.set(speed);
	}

	@Override
	public double getPositionInDegrees() {
		// TODO Auto-generated method stub
		return climbMotor.getPosition().getValue().in(Degrees);
	}
}
