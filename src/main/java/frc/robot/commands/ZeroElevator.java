package frc.robot.commands;

import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class ZeroElevator extends Command{
    private static ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();

    //     public static Command zeroElevator() {
    //     return new FunctionalCommand(
    //             () -> elevatorSubsystem.rightMotorMaster.setVoltage(-2),
    //             () -> {
    //             },
    //             (interrupted) -> {
    //                 elevatorSubsystem.setVoltage(0);
    //             },
    //             elevatorSubsystem::isAtBottom,
    //             elevatorSubsystem
    //     );
    // }
    
    @Override
    public void execute(){
        elevatorSubsystem.rightMotorMaster.setVoltage(-2);
        elevatorSubsystem.leftMotorFollower.setControl(new Follower(elevatorSubsystem.rightMotorMaster.getDeviceID(), true));
    }
    public static void stopZero(){
        elevatorSubsystem.rightMotorMaster.setVoltage(0);
        elevatorSubsystem.leftMotorFollower.setControl(new Follower(elevatorSubsystem.rightMotorMaster.getDeviceID(), true));
        elevatorSubsystem.setPosition(0);
    }

    @Override
    public void end(boolean interrupted){
        elevatorSubsystem.setVoltage(0);
    }

    @Override
    public boolean isFinished(){
        return elevatorSubsystem.isAtBottom();
    }
}
