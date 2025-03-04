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
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;

public final class Constants {

    public static final RobotVersions curRobot = RobotVersions.COMPBOT;

    public static CommandPS4Controller driver = new CommandPS4Controller(0);
    public static CommandXboxController operator = new CommandXboxController(1);

    public static final double L0 = ElevatorConstants.ELEVATORBASEHEIGHT;
    public static final double ELEVATORZEROHEIGHT = 25;
    public static final double L1 = 21;
    public static final double L2 = 29; // 31.25
    public static final double L3 = 43;
    public static final double L4 = 67;
    /*
     * joystick.triangle().onTrue(new ElevatorCommands(elevator, 14.5));
     * joystick.circle().onTrue(new ElevatorCommands(elevator, 20));
     * joystick.cross().onTrue(new ElevatorCommands(elevator, 31.5));
     * joystick.square().onTrue(new ElevatorCommands(elevator, 45.5));
     * joystick.L1().onTrue(new ElevatorCommands(elevator, 71.1));
     */

    public static class PhotonVisionConstants {
        public static PhotonCamera m_Camera = new PhotonCamera("logitech");

        // DESIRED CONSTANTS IN METERS FIXME
        public static final double DES_REEF_DISTANCE = 0.05;
        // public static final double DES_HUMAN_DISTANCEX = 0.381;
        public static final double DES_HUMAN_DISTANCE_Y_LEFT = -0.2;
        public static final double DES_HUMAN_DISTANCE_Y_RIGHT = 0.2;

        public static final double DES_ANGLE = 0;

        // public static final double CAMERA_HEIGHT = 0.3175; //FIXME
        // public static final double CAMERA_ANGLE = 0.785398;

        // APRILTAGS
        // public static final double APRILTAG_7_HEIGHT = 0.31; //FIXME

        // How far from the target we want to be
        // public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(new
        // Translation3d(0, -0.375, -0.0762), new Rotation3d(0, 0, CAMERA_ANGLE));
        // //FIXME
        // public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(new
        // Translation3d(0, 0.375, 0.0762), new Rotation3d(0, 0, CAMERA_ANGLE)); //FIXME
    }

    public static class ElevatorConstants {
        public static final int LIMITSWITCHID = 0;

        public static final int ELEVATORLEFTID = 6;
        public static final int ELEVATORRIGHTID = 5;

        public static final double ELEVATORBASEHEIGHT = 15;
        public static final double ELEVATORBASESECONDHEIGHT = 25;
        public static final double INCHPERROTATION = (ELEVATORBASESECONDHEIGHT - ELEVATORBASEHEIGHT) / 5.0;

        public static final double MAXROTATIONS = 26; // FIXME

        public static TalonFXConfiguration ELEVATORCONFIG = new TalonFXConfiguration();
        static {
            ELEVATORCONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            // ELEVATORCONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            ELEVATORCONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            ELEVATORCONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 26;
            ELEVATORCONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            ELEVATORCONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

            ELEVATORCONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;

            // ELEVATORCONFIG.Feedback.SensorToMechanismRatio = 0.2; //FIXME THIS MAY BE 5

            ELEVATORCONFIG.Slot0.kG = 0.3; // .35
            ELEVATORCONFIG.Slot0.kS = 0.48; // .375
            ELEVATORCONFIG.Slot0.kV = 0.001; // .001
            ELEVATORCONFIG.Slot0.kA = 0.0;
            ELEVATORCONFIG.Slot0.kP = 0.7;// 0.4
            ELEVATORCONFIG.Slot0.kI = 0.07;// 0.1
            ELEVATORCONFIG.Slot0.kD = 0.01;// 0.02

            ELEVATORCONFIG.MotionMagic.MotionMagicCruiseVelocity = 125;// using 100 nonr 125 some 200 some
            ELEVATORCONFIG.MotionMagic.MotionMagicAcceleration = 250;
            ELEVATORCONFIG.MotionMagic.MotionMagicExpo_kV = 0.5;
            // ELEVATORCONFIG.Slot0.StaticFeedforwardSign =
            // StaticFeedforwardSignValue.UseClosedLoopSign;
        }

        public static TalonFXConfiguration ELEVATOR_COAST_CONFIG = new TalonFXConfiguration();
        static {
            ELEVATOR_COAST_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }

        public static final double ELEVATORDEADBAND = 0.3;
    }

    public static class CoralManipulatorConstants {
        public static final int BAGMOTOR = 10;
    }

    public static class ClimbConstants {
        public static final int MOTORONE = 13; // FIXEME
        public static final int MOTORTWO = 7; // FIXEME
        public static TalonFXConfiguration CLIMBCONFIG = new TalonFXConfiguration();
        static {
            CLIMBCONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        }
    }
    // public static class GroundIntake{
    // public static final int ROTATEMOTOR = ;
    // public static final int DRIVEMOTOR = ;
    // }
}
