package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralManipulator extends SubsystemBase{
    // private TalonFX bagMotor;
    private TalonSRX bagMotor;

    public CoralManipulator(){
        bagMotor = new TalonSRX(Constants.CoralManipulatorConstants.BAGMOTOR);
    }

    public void setSpeed(double speed){
        bagMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }
}
