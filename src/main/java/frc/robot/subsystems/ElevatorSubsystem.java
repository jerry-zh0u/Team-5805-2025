package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase{
    private TalonFX leftMotorFollower, rightMotorMaster;

    private double lastPos;
    private int onStall;

    private DigitalInput limitSwitch;

    VoltageOut voltageRequest = new VoltageOut(0);

    MotionMagicVoltage motionRequest;
    
    public ElevatorSubsystem(){
        leftMotorFollower = new TalonFX(Constants.ElevatorConstants.ELEVATORLEFTID);
        rightMotorMaster = new TalonFX(Constants.ElevatorConstants.ELEVATORRIGHTID);

        lastPos = Constants.ElevatorConstants.ELEVATORBASEHEIGHT;
        motionRequest = new MotionMagicVoltage(0);

        limitSwitch = new DigitalInput(Constants.ElevatorConstants.LIMITSWITCHID);

        rightMotorMaster.getConfigurator().apply(Constants.ElevatorConstants.ELEVATORCONFIG);
        leftMotorFollower.getConfigurator().apply(Constants.ElevatorConstants.ELEVATORCONFIG);

        onStall = 0;
    }

    public void setPosition(double height){
        lastPos = height;

        rightMotorMaster.setControl(motionRequest.withPosition(convertDistRotation(height)));
        leftMotorFollower.setControl(new Follower(rightMotorMaster.getDeviceID(), true));
    }

    public double getPosition(){
        return rightMotorMaster.getPosition().getValueAsDouble();
    }

    public double getHeight(){
        return rightMotorMaster.getPosition().getValueAsDouble() * Constants.ElevatorConstants.INCHPERROTATION + Constants.ElevatorConstants.ELEVATORBASEHEIGHT;
    }

    public void zero(){
        rightMotorMaster.setPosition(0.0);
        leftMotorFollower.setPosition(0.0);

        setVoltage(0);

        lastPos = Constants.ElevatorConstants.ELEVATORBASEHEIGHT;
        // rightMotorMaster.setControl(motionRequest.withPosition(convertDistRotation(lastPos)));
        System.err.println("Zero Run" + " " + rightMotorMaster.getPosition().getValueAsDouble());
    }

    private double convertDistRotation(double height){
        return (height - Constants.ElevatorConstants.ELEVATORBASEHEIGHT)/Constants.ElevatorConstants.INCHPERROTATION;
    }

    public void setMode(boolean coastMode){
        if(coastMode){
            rightMotorMaster.getConfigurator().apply(Constants.ElevatorConstants.ELEVATOR_COAST_CONFIG);
            leftMotorFollower.getConfigurator().apply(Constants.ElevatorConstants.ELEVATOR_COAST_CONFIG);
        }else{
            rightMotorMaster.getConfigurator().apply(Constants.ElevatorConstants.ELEVATORCONFIG);
            leftMotorFollower.getConfigurator().apply(Constants.ElevatorConstants.ELEVATORCONFIG);
        }
    }

    public void setVoltage(double amt){
        rightMotorMaster.setControl(voltageRequest.withOutput(amt));
        leftMotorFollower.setControl(voltageRequest.withOutput(amt));
        // leftMotorFollower.setControl(new Follower(rightMotorMaster.getDeviceID(), true));
    }

    public boolean atBottom(){
        return onStall >= 1;
    }

    public boolean getLimitSwitch(){
        return limitSwitch.get();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Position", getHeight());
        SmartDashboard.putNumber("Elevator Desired Position", lastPos);

        SmartDashboard.putNumber("Raw Encoder Readings", rightMotorMaster.getPosition().getValueAsDouble());

        if(rightMotorMaster.getSupplyCurrent().getValueAsDouble() > 3.5){
            onStall++;
        }else{
            onStall = 0;
        }

        SmartDashboard.putNumber("Stall Counter", onStall);
    }
}