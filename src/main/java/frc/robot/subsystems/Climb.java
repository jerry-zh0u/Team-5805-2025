package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase{
    private TalonFX motor1, motor2;
    private SlewRateLimiter limiter;

    public Climb(){
        motor1 = new TalonFX(Constants.ClimbConstants.MOTORONE);

        motor2 = new TalonFX(Constants.ClimbConstants.MOTORTWO);

        motor1.getConfigurator().apply(Constants.ClimbConstants.CLIMBCONFIG);
        motor2.getConfigurator().apply(Constants.ClimbConstants.CLIMBCONFIG);

        // motor2.setControl(new Follower(motor1.getDeviceID(), true));

        // limiter = new SlewRateLimiter(0.5);
    }

    public void setSpeed(double speed){
        motor1.set(speed);
        motor2.set(speed);
        // motor1.set(limiter.calculate(speed));
        // motor2.set(limiter.calculate(speed));
        // motor2.setControl(new Follower(motor1.getDeviceID(), false));
    }

    public void setZero(){
        System.err.println("Climb Zeroed");
        motor1.setPosition(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Climb Ticks", motor1.getPosition().getValueAsDouble());
        
        // if(motor1.getPosition().getValueAsDouble() < -75){
        //     motor1.setVoltage(0);
        //     motor2.setVoltage(0);
        // }
    }
}
