package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitCommand extends Command {
    private double timeToWait;
    private double startTime;
    
    public WaitCommand(double waitTime){
       timeToWait = waitTime;
    }

    @Override
    public void initialize(){
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished(){
        return Timer.getFPGATimestamp() - startTime >= timeToWait;
    }
}
