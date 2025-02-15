package frc.robot;

import frc.robot.constants.RobotVersions;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generated.TunerConstants;

public final class Constants{
    public static final RobotVersions curRobot = RobotVersions.COMPBOT;
    public static final Distance L0 = Inches.of(0);
    public static final Distance L1 = Inches.of(1);
    public static final Distance L2 = Inches.of(1.6);
    public static final Distance L3 = Inches.of(3);
    public static final Distance L4 = Inches.of(4);

    public static class PhotonVisionConstants{
        public static PhotonCamera m_Camera = new PhotonCamera("logitech");

        //DESIRED CONSTANTS IN METERS FIXME
        public static final double DES_REEF_DISTANCE = 0.05;
        public static final double DES_HUMAN_DISTANCEX = 0.05;
        public static final double DES_HUMAN_DISTANCEY = 0;
        public static final double DES_ANGLE = 0;

        public static final double CAMERA_HEIGHT = 0.2; //FIXME
        public static final double CAMERA_ANGLE = 0.785398;
        
        //APRILTAGS
        public static final double APRILTAG_7_HEIGHT = 0.31; //FIXME

        // How far from the target we want to be
        public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0, -0.375, -0.0762), new Rotation3d(0, 0, CAMERA_ANGLE)); //FIXME
        public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(new Translation3d(0, 0.375, 0.0762), new Rotation3d(0, 0, CAMERA_ANGLE)); //FIXME
    }
    public static class ElevatorConstants{
        public static final int ELEVATORLEFTID = 5;
        public static final int ELEVATORRIGHTID = 6;

        public static TalonFXConfiguration ELEVATORCONFIG = new TalonFXConfiguration();
            static{                
                ELEVATORCONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                // ELEVATORCONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

                ELEVATORCONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
                ELEVATORCONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Inches.of(100).in(Inches);
                ELEVATORCONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
                ELEVATORCONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Inches.of(0).in(Inches);

                // ELEVATORCONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;

                ELEVATORCONFIG.Feedback.SensorToMechanismRatio = 5; //FIXME THIS MAY BE 5

                ELEVATORCONFIG.Slot0.kG = 0.3;
                ELEVATORCONFIG.Slot0.kS = 0.4;
                ELEVATORCONFIG.Slot0.kV = 0.001;
                ELEVATORCONFIG.Slot0.kA = 0.0;
                ELEVATORCONFIG.Slot0.kP = 2;
                ELEVATORCONFIG.Slot0.kI = 0.0;
                ELEVATORCONFIG.Slot0.kD = 0.0;

                ELEVATORCONFIG.MotionMagic.MotionMagicCruiseVelocity = 10;
                ELEVATORCONFIG.MotionMagic.MotionMagicAcceleration = 3;
                ELEVATORCONFIG.MotionMagic.MotionMagicExpo_kV = 0.12;
                // ELEVATORCONFIG.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
            }

        public static final Distance ELEVATORDEADBAND = Inches.of(.5);
    }
    public static class CoralManipulatorConstants{
        public static final int BAGMOTOR = 10; //FIXME
    }
}
