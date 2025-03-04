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

public class ElevatorCommands extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private double desiredDist;
    // private boolean finished;

    public ElevatorCommands(ElevatorSubsystem _elevator, double height) {
        this.elevatorSubsystem = _elevator;
        this.desiredDist = height;
        // finished = false;
    }

    @Override
    public void execute() {
        // System.err.println("Elevator Run");
        // finished = false;
        elevatorSubsystem.setPosition(desiredDist);
    }

    @Override
    public boolean isFinished() {
        if (elevatorSubsystem.getPosition() >= Constants.ElevatorConstants.MAXROTATIONS || Math
                .abs(elevatorSubsystem.getHeight() - desiredDist) <= Constants.ElevatorConstants.ELEVATORDEADBAND) {
            System.err.println("+++++ Finished");
            // finished = true;
            return true;
        }
        return false;
    }

    // @Override
    // public void periodic(){

    // }
    // }

    // Cp,ommadnw
    // @Override
    public void end(boolean interrupted) {
        // return true;
    }
}
