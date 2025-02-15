package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase{
    private TalonFX leftMotorFollower, rightMotorMaster;

    private Distance lastPos;
    private Distance curLeftPos, curRightPos;

    VoltageOut voltageRequest = new VoltageOut(0);

    MotionMagicVoltage motionRequest;
    
    public ElevatorSubsystem(){
        leftMotorFollower = new TalonFX(Constants.ElevatorConstants.ELEVATORLEFTID);
        rightMotorMaster = new TalonFX(Constants.ElevatorConstants.ELEVATORRIGHTID);

        lastPos = Inches.of(0);
        motionRequest = new MotionMagicVoltage(0);

        rightMotorMaster.getConfigurator().apply(Constants.ElevatorConstants.ELEVATORCONFIG);
        leftMotorFollower.getConfigurator().apply(Constants.ElevatorConstants.ELEVATORCONFIG);
    }

    public void setPosition(Distance height){
        lastPos = height;

        rightMotorMaster.setControl(motionRequest.withPosition(height.in(Inches)));
        // rightMotorMaster.setControl(motionRequest.withPosition(height.in(Inches)));
        leftMotorFollower.setControl(new Follower(rightMotorMaster.getDeviceID(), true));
    }

    public double getHeight(){
        return rightMotorMaster.getPosition().getValueAsDouble();
    }

    public void zero(){
        System.err.println("Zero Run");
        lastPos = Inches.of(0);
        rightMotorMaster.setPosition(Constants.L0.in(Inches));
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Position", getHeight());
        SmartDashboard.putNumber("Elevator Desired Position", lastPos.in(Inches));
    }
}
