package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommands extends Command{
    private ElevatorSubsystem elevator;
    private Distance desiredDist;

    public ElevatorCommands(ElevatorSubsystem _elevator, Distance height){
        this.elevator = _elevator;
        this.desiredDist = height;
    }

    @Override
    public void execute(){
        System.err.println("======" + desiredDist.toString());
        elevator.setPosition(desiredDist);
    }

    @Override
    public boolean isFinished(){
        if(Math.abs(elevator.getHeight() - desiredDist.in(Inches)) <= Constants.ElevatorConstants.ELEVATORDEADBAND.in(Inches)){
            System.err.println("+++++");
            return true;
        }
        return false;
    }

    // @Override
    // public void end(boolean interrupted){
    //     return true;
    // }
}
