package frc.robot.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralManipulator;

public class CoralManipulatorGo extends Command {
    private CoralManipulator elevator;
    private double speed;

    public CoralManipulatorGo(CoralManipulator _elevator, double _speed){
        this.elevator = _elevator;
        speed = _speed;
    }

    @Override
    public void execute(){
        elevator.setSpeed(speed);
    }
}
