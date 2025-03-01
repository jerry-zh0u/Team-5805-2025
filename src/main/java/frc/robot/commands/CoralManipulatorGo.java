package frc.robot.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralManipulator;

public class CoralManipulatorGo extends Command {
    private CoralManipulator coral;
    private double speed;

    public CoralManipulatorGo(CoralManipulator _coral, double _speed){
        this.coral = _coral;
        speed = _speed;
    }

    @Override
    public void execute(){
        coral.setSpeed(speed);
    }

    @Override
    public boolean isFinished(){
        System.err.println("Coral Manipulator is Finished");
        return true;
    } 
}
