package frc.robot;

import frc.robot.constants.RobotVersions;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public final class Constants{
    public static final RobotVersions curRobot = RobotVersions.COMPBOT;
    public static class PhotonVisionConstants{
        public static PhotonCamera m_Camera = new PhotonCamera("April"); //FIXME

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
}
