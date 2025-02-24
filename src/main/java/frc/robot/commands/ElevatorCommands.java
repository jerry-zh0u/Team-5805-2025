package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommands extends Command{
    private ElevatorSubsystem elevator;
    private double desiredDist;

    public ElevatorCommands(ElevatorSubsystem _elevator, double height){
        this.elevator = _elevator;
        this.desiredDist = height;
    }

    @Override
    public void execute(){
        elevator.setPosition(desiredDist);
    }

    @Override
    public boolean isFinished(){
        if(Math.abs(elevator.getHeight() - desiredDist) <= Constants.ElevatorConstants.ELEVATORDEADBAND){
            System.err.println("+++++ Finished");
            return true;
        }
        return false;
    }

    // @Override
    // public void end(boolean interrupted){
    //     return true;
    // }
}
