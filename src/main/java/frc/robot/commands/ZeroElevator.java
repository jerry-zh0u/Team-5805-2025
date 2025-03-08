package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
// import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TagalongElevatorSystem;
import tagalong.subsystems.micro.Elevator;

public class ZeroElevator extends Command {
    private TagalongElevatorSystem elevator;

    public ZeroElevator(TagalongElevatorSystem _elevator) {
        this.elevator = _elevator;
    }

    @Override
    public void execute() {
        elevator.setPower(0.0);
    }

    @Override
    public boolean isFinished() {
        return elevator.getLimitSwitch();
    }

    @Override
    public void end(boolean interrupted) {

        System.err.println("Elevator has zeroed");

        elevator.zero();
    }
}
