package frc.robot.subsystems.simplearm;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;


public interface ArmIO {
    @AutoLog
    public class ArmIOInputs {
        public boolean armConnected = false;
        public double armPositionRad = 0.0;
        public double armAppliedVolts = 0.0;
        public double[] armCurrentAmps = new double[]{};
        public AngularVelocity armVelocity = RadiansPerSecond.of(0);
    }
    
    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setArmVelocity(AngularVelocity velocityRadPerSec) {}

    public default void setArmVoltage(double volts) {}


}
    

