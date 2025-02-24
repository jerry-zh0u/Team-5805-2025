package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ElevatorSubsystem;

public class ClimbCommands extends Command{
    private Climb climb;
    private double speed;

    public ClimbCommands(Climb _climb, double _speed){
        this.climb = _climb;
        this.speed = _speed;
    }

    @Override
    public void execute(){
        climb.setSpeed(speed);
    }
}
