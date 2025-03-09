package frc.robot.subsystems.algae;

import static frc.robot.subsystems.algae.AlgaeConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class AlgaeIOSpark implements AlgaeIO {
	SparkMax motor = new SparkMax(algaeMotorID, MotorType.kBrushless);

	public AlgaeIOSpark() {}
}
