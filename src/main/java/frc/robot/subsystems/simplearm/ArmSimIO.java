package frc.robot.subsystems.simplearm;

import static edu.wpi.first.units.Units.KilogramMetersPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.MomentOfInertiaUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ArmSimIO implements ArmIO{
    SimulatedMotorController.GenericMotorController armMotor;
    public MomentOfInertia moi = KilogramSquareMeters.of(0.1297660794);
    SimMotorConfigs configs = new SimMotorConfigs(DCMotor.getNeo550(1), 0, moi, null);
    MapleMotorSim motorSim = new MapleMotorSim(configs);
    double armAppliedVolts = 0.0;

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        motorSim.update(0.02);
    }


}
