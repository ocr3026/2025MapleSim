package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
	private final ElevatorSim elevatorSim = new ElevatorSim(kV, kA, gearbox, minPosition.in(Meters), maxPosition.in(Meters), true, minPosition.in(Meters));

	
}
