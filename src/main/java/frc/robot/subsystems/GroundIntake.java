package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GroundIntake extends SubsystemBase{
    // private TalonFX bagMotor;
    private TalonFX rotateMotor, driveMotor;
    private DigitalInput beamBreak;

    public GroundIntake(){
        // rotateMotor = new TalonFX(Constants.GroundIntake.ROTATEMOTOR);
        // rotateMotor = new TalonFX(Constants.GroundIntake.DRIVEMOTOR);
    }

    public void setSpeed(double speed){
        

    }

    @Override
    public void periodic(){
        // if(beamBreak.get()){
        //     bagMotor.set(TalonSRXControlMode.PercentOutput, 0.2);
        // }
        // else{
        //     bagMotor.set(TalonSRXControlMode.PercentOutput, 0);
        // }
    }
}
