package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public final class Constants{
    public static class PhotonVisionConstants{
        public static PhotonCamera m_Camera = new PhotonCamera("April");

       // Constants such as camera and target height stored. Change per robot and goal!
        public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
        public static final double TARGET_HEIGHT_METERS = Units.feetToMeters(4.8);

        // Angle between horizontal and the camera.
        public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(47);

        // How far from the target we want to be
        public static final double GOAL_RANGE_METERS = Units.feetToMeters(8);
        public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(new Translation3d(null, null, null), new Rotation3d()); //FIXME
    }
}