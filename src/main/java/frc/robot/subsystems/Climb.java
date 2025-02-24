package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase{
    private TalonFX motor1, motor2;

    public Climb(){
        motor1 = new TalonFX(Constants.ClimbConstants.MOTORONE);
        motor2 = new TalonFX(Constants.ClimbConstants.MOTORTWO);
    }

    public void setSpeed(double speed){
        motor1.set(speed);
        motor2.set(speed);
        // motor2.setControl(new Follower(motor1.getDeviceID(), false));
    }
}
