package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public final class Constants{
    public static class LimeLightConstants{
       // Constants such as camera and target height stored. Change per robot and goal!
        public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
        public static final double TARGET_HEIGHT_METERS = Units.feetToMeters(4.8);

        // Angle between horizontal and the camera.
        public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(47);

        // How far from the target we want to be
        public static final double GOAL_RANGE_METERS = Units.feetToMeters(8);

    }
}