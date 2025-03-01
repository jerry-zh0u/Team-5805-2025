package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.WaitCommand;

public class CoralManipulator extends SubsystemBase{
    // private TalonFX bagMotor;
    private TalonSRX bagMotor;
    private DigitalInput beamBreak;

    private boolean run;
    private boolean lastRead;

    public CoralManipulator(){
        bagMotor = new TalonSRX(Constants.CoralManipulatorConstants.BAGMOTOR);
        beamBreak = new DigitalInput(1);

        run = false;
        lastRead = false;
    }

    public void setSpeed(double speed){
        if(speed == 0){
            run = false;
        }else{
            run = true;
        }

        bagMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Beam Break Sensor", beamBreak.get());
        SmartDashboard.putBoolean("Currently Running", run);

        if(!run){
            if(beamBreak.get()){
                bagMotor.set(TalonSRXControlMode.PercentOutput, -0.25);
            }
            else{
                if(lastRead){
                    new WaitCommand(0.01);
                }
                bagMotor.set(TalonSRXControlMode.PercentOutput, 0);
            }
        }
        lastRead = beamBreak.get();
    }
}
