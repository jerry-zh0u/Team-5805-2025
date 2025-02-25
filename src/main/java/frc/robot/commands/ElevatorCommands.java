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


public class ElevatorCommands extends Command{
    private static ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
    private double desiredDist;

    public ElevatorCommands(ElevatorSubsystem _elevator, double height){
        this.elevatorSubsystem = _elevator;
        this.desiredDist = height;
    }

    @Override
    public void execute(){
        elevatorSubsystem.setPosition(desiredDist);
    }

    @Override
    public boolean isFinished(){
        if(Math.abs(elevatorSubsystem.getHeight() - desiredDist) <= Constants.ElevatorConstants.ELEVATORDEADBAND){
            System.err.println("+++++ Finished");
            return true;
        }
        return false;
    }

    
    public static Command zeroSubsystems() {
        return new SequentialCommandGroup(
                new ZeroElevator(),
                new WaitCommand(.3),
                new InstantCommand(() -> elevatorSubsystem.setPosition(15))
        );
    }
}

    // @Override
    // public void end(boolean interrupted){
    //     return true;
    // }
// }
