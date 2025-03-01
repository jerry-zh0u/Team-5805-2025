package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;


public class ElevatorBounce extends Command{
    private ElevatorSubsystem elevatorSubsystem;
    private double desiredDist;
    private static int count = 0;

    public ElevatorBounce(ElevatorSubsystem _elevator, double height){
        this.elevatorSubsystem = _elevator;
        this.desiredDist = height;
    }

    @Override
    public void execute(){
        count = 0;

        new ElevatorCommands(elevatorSubsystem, desiredDist - 2);
        new ElevatorCommands(elevatorSubsystem, desiredDist + 2);
        new ElevatorCommands(elevatorSubsystem, desiredDist);
        isFinished();

        count++;
    }

    @Override
    public boolean isFinished(){
        if(count >= 1){
            return true;
        }
        return false;
    }
}

    // @Override
    // public void end(boolean interrupted){
    //     return true;
    // }
// }
