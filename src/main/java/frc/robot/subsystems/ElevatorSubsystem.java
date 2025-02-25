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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase{
    public static ElevatorSubsystem elevatorSubsystem;
    public TalonFX leftMotorFollower, rightMotorMaster;

    private Follower follower;

    private double lastPos;

    private int currentSpike;
    private int noCurrentSpike;

    VoltageOut voltageRequest = new VoltageOut(0);

    MotionMagicVoltage motionRequest;
    
    public ElevatorSubsystem(){
        // leftMotorFollower = new TalonFX(Constants.ElevatorConstants.ELEVATORLEFTID);
        rightMotorMaster = new TalonFX(Constants.ElevatorConstants.ELEVATORRIGHTID);
        rightMotorMaster.getConfigurator().apply(Constants.ElevatorConstants.ELEVATORCONFIG);

        configureFollowerMotor(Constants.ElevatorConstants.ELEVATORLEFTID, true);

        lastPos = 15;
        motionRequest = new MotionMagicVoltage(0);

        // leftmotorfollower.set(Follower, rightMotorMaster);
        // leftMotorFollower.setControl(new Follower(rightMotorMaster.getDeviceID(), true));
        // leftMotorFollower.getConfigurator().apply(Constants.ElevatorConstants.ELEVATORCONFIG);
    }

    public void configureFollowerMotor(int followerMotorId, boolean opposeMasterDirection) {
        leftMotorFollower = new TalonFX(followerMotorId);
        follower = new Follower(rightMotorMaster.getDeviceID(), opposeMasterDirection);

        leftMotorFollower.setControl(follower);
    }

    public void setPosition(double height){
        lastPos = height;
        //CHECK THE UNITS
        // rightMotorMaster.set(height);
        rightMotorMaster.setControl(motionRequest.withPosition(convertDistRotation(height)));
        // rightMotorMaster.setControl(motionRequest.withPosition(10));


        // leftMotorFollower.setControl(new Follower(rightMotorMaster.getDeviceID(), true));
    }

    public double getHeight(){
        return rightMotorMaster.getPosition().getValueAsDouble() * Constants.ElevatorConstants.INCHPERROTATION + Constants.ElevatorConstants.ELEVATORBASEHEIGHT;
    }

    public void zero(){
        lastPos = Constants.ElevatorConstants.ELEVATORBASEHEIGHT;
        // rightMotorMaster.setControl(motionRequest.withPosition(convertDistRotation(lastPos)));
        rightMotorMaster.setPosition(0.0);
        System.err.println("Zero Run" + " " + rightMotorMaster.getPosition().getValueAsDouble());
    }

    public void setVoltage(double output) {
        rightMotorMaster.setVoltage(output);
    }

    private double convertDistRotation(double height){
        return (height - Constants.ElevatorConstants.ELEVATORBASEHEIGHT)/Constants.ElevatorConstants.INCHPERROTATION;

        // return 13.5 +5x = height
    }

    public void setMode(boolean coastMode){
        if(coastMode){
            rightMotorMaster.getConfigurator().apply(Constants.ElevatorConstants.ELEVATOR_COAST_CONFIG);
            // leftMotorFollower.getConfigurator().apply(Constants.ElevatorConstants.ELEVATOR_COAST_CONFIG);
        }else{
            rightMotorMaster.getConfigurator().apply(Constants.ElevatorConstants.ELEVATORCONFIG);
            // leftMotorFollower.getConfigurator().apply(Constants.ElevatorConstants.ELEVATORCONFIG);
        }
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Position", getHeight());
        SmartDashboard.putNumber("Elevator Desired Position", lastPos);
        super.periodic();

        if (rightMotorMaster.getSupplyCurrent().getValueAsDouble() > 3.5) 
            currentSpike++;
        else noCurrentSpike++;

        if (noCurrentSpike >= 3) {
            currentSpike = 0;
            noCurrentSpike = 0;
        }

        SmartDashboard.putNumber("Stall Count Current Spike", currentSpike);
        SmartDashboard.putNumber("No Stall Count Current Spike", noCurrentSpike);
    }

    public static ElevatorSubsystem getInstance() {
        if (elevatorSubsystem == null) {
            elevatorSubsystem = new ElevatorSubsystem();
        }
        return elevatorSubsystem;
    }

    public void testVoltage() {
        rightMotorMaster.setVoltage(-1);
    }

    public boolean isAtBottom() {
        return currentSpike >= 1;
    }
}
