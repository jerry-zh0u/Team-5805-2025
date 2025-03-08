package frc.robot.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralManipulator;

public class AutoCoralManipulator extends Command {
    private CoralManipulator coral;
    private WaitCommand wait;
    private double speed;

    public AutoCoralManipulator(CoralManipulator _coral, double _speed) {
        this.coral = _coral;
        speed = _speed;
        wait = new WaitCommand(0.3);
    }

    @Override
    public void execute() {
        coral.setSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return wait.isFinished();
    }
}
