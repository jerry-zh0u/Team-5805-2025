package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ZeroElevator extends Command{
    private ElevatorSubsystem elevator;

    public ZeroElevator(ElevatorSubsystem _elevator){
        this.elevator = _elevator;
    }

    @Override
    public void execute(){
        elevator.setVoltage(-1);
    }

    @Override
    public boolean isFinished(){
        return elevator.getLimitSwitch();
    }

    @Override
    public void end(boolean interrupted){

        System.err.println("Elevator has zeroed");

        elevator.setVoltage(0);
        elevator.zero();
    }
}
