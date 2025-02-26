package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralManipulator extends SubsystemBase{
    // private TalonFX bagMotor;
    private TalonSRX bagMotor;
    private DigitalInput beamBreak;

    public CoralManipulator(){
        bagMotor = new TalonSRX(Constants.CoralManipulatorConstants.BAGMOTOR);
        // beamBreak = new DigitalInput(0);
    }

    public void setSpeed(double speed){
        bagMotor.set(TalonSRXControlMode.PercentOutput, speed);
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
