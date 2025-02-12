package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import static frc.robot.RobotContainer.elevatorSubsystem;
import static frc.robot.Constants.*;

public class ElevatorCommands extends Command{
    ElevatorSubsystem subsystem;
    public int enumID = 0;

    public ElevatorCommands() {
        subsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        switch(pos) {
            case HIGH:
            break;
            case MID:
            break;
            case LOW:
            break;
            case HOME:
            break;
            case INTAKE:
            break;
        }
    }

}
