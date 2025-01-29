package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import frc.robot.util.SparkUtil;

public class ModuleIOSim implements ModuleIO {
	private final SwerveModuleSimulation moduleSimulation;
	private final SimulatedMotorController.GenericMotorController driveMotor;
	private final SimulatedMotorController.GenericMotorController turnMotor;

	private boolean driveClosedLoop = false;
	private boolean turnClosedLoop = false;
	private final PIDController driveController = new PIDController(driveSimP, 0, driveSimD);
	private final SimpleMotorFeedforward driveFFController = new SimpleMotorFeedforward(driveSimKs, driveSimKv);
	private final PIDController turnController = new PIDController(turnSimP, 0, turnSimD);
	private double driveFFVolts = 0.0;
	private double driveAppliedVolts = 0.0;
	private double turnAppliedVolts = 0.0;

	public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
		this.moduleSimulation = moduleSimulation;
		this.driveMotor = moduleSimulation.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(driveMotorCurrentLimit));
		this.turnMotor = moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(turnMotorCurrentLimit));
		turnController.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		if(driveClosedLoop) {
			driveAppliedVolts = driveFFVolts + driveController.calculate(moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond));
		} else {
			driveController.reset();
		}

		if(turnClosedLoop) {
			turnAppliedVolts = turnController.calculate(moduleSimulation.getSteerAbsoluteFacing().getRadians());
		} else {
			turnController.reset();
		}

		driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
		turnMotor.requestVoltage(Volts.of(turnAppliedVolts));

		inputs.driveConnected = true;
		inputs.drivePosition = Meters.of(moduleSimulation.getDriveWheelFinalPosition().in(Radians) * wheelRadius.in(Meters));
		inputs.driveVelocity = MetersPerSecond.of(moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond) * wheelRadius.in(Meters));
		inputs.driveAppliedVolts = driveAppliedVolts;
		inputs.driveCurrentAmps = Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps));

		inputs.turnConnected = true;
		inputs.turnPosition = moduleSimulation.getSteerAbsoluteFacing();
		inputs.turnVelocity = moduleSimulation.getSteerAbsoluteEncoderSpeed();
		inputs.turnAppliedVolts = turnAppliedVolts;
		inputs.turnCurrentAmps = Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));

		inputs.odometryTimestamps = SparkUtil.getSimulationOdometryTimeStamps();

		Angle[] angles = moduleSimulation.getCachedDriveWheelFinalPositions();
		Distance[] distances = new Distance[angles.length];
		for(int i = 0; i < angles.length; i++) {
			distances[i] = Meters.of(angles[i].in(Radians) * wheelRadius.in(Meters));
		}
		inputs.odometryDrivePositions = distances;
		inputs.odometryTurnPositions = moduleSimulation.getCachedSteerAbsolutePositions();
	}

	@Override
    public void setDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    @Override
    public void setDriveVelocity(LinearVelocity velocity) {
        driveClosedLoop = true;
        driveFFVolts = driveFFController.calculate(velocity.in(MetersPerSecond));
        driveController.setSetpoint(velocity.in(MetersPerSecond));
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnController.setSetpoint(rotation.getRadians());
    }
}
