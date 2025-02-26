package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import org.photonvision.PhotonPoseEstimator;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    ShuffleboardTab controlboardTab = Shuffleboard.getTab("Competition HUD");

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    public static final Field2d _field = new Field2d();
    private AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private PhotonCamera cam = Constants.PhotonVisionConstants.m_Camera;
    private PhotonPoseEstimator camPoseEstimator = new PhotonPoseEstimator( layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.PhotonVisionConstants.ROBOT_TO_CAMERA); // TODO Jerry, test different pose strageies. In general just use multitag

    private static PhotonPipelineResult cur = null;

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        // controlboardTab.add("Field", _field).withSize(11, 5).withPosition(1, 1);
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation  The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(5, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        // SmartDashboard.putNumber("Pose X", getState().Pose.getX());
        // SmartDashboard.putNumber("Pose Y", getState().Pose.getY());
        SmartDashboard.putNumber("Velocity 1", getState().ModuleStates[0].speedMetersPerSecond);
        SmartDashboard.putNumber("Velocity 2", getState().ModuleStates[1].speedMetersPerSecond);
        SmartDashboard.putNumber("Velocity 3", getState().ModuleStates[2].speedMetersPerSecond);
        SmartDashboard.putNumber("Velocity 4", getState().ModuleStates[3].speedMetersPerSecond);
        // SmartDashboard.putNumber("Pose Y", getState().Pose.getY());
        // _field.setRobotPose(getState().Pose.getX(), getState().Pose.getY(), getState().Pose.getRotation());

        //VISION
        var results = cam.getAllUnreadResults();
        // var timestamp = Utils.getCurrentTimeSeconds();
        // System.err.println(results.size() + " ==========");
        // if(!results.isEmpty()){
        //     System.err.println("..............");
        //     cur = results.get(results.size() - 1);

            // can potentially run through every result rather than just the most recent for more accurate representation of data
            // var photonEstimate = camPoseEstimator.update(cur).orElse(null);
            // if( photonEstimate != null){
            //     System.err.println("/////////" + " " + photonEstimate.estimatedPose.toPose2d().getX() + " " + photonEstimate.estimatedPose.toPose2d().getY() + " " + photonEstimate.timestampSeconds);
            //     // addVisionMeasurement(photonEstimate.estimatedPose.toPose2d(), photonEstimate.timestampSeconds);
            //     addVisionMeasurement(photonEstimate.estimatedPose.toPose2d(), photonEstimate.timestampSeconds);
            // }
            // var results = cam.getAllUnreadResults();
            var timestamp = Utils.getCurrentTimeSeconds();

        if(!results.isEmpty()){
            cur = results.get(results.size() - 1);
            
            // var best = cur.getBestTarget();
            // if(best != null){
            //     var bestID = best.getFiducialId();

            //     // System.err.println(bestID + " ==== Found");

            //     Optional<Pose3d> tagPose = layout.getTagPose(bestID);
            //     if(best.getPoseAmbiguity() <= 0.2 && bestID >= 0 && tagPose.isPresent()){
            //         var targetPose = tagPose.get();
            //         Transform3d camToTarget = best.getBestCameraToTarget();
            //         Pose3d camPose = targetPose.transformBy(camToTarget.inverse());
    
            //         var visionMeasure = camPose.transformBy(Constants.PhotonVisionConstants.CAMERA_TO_ROBOT);
            //         addVisionMeasurement(visionMeasure.toPose2d(), timestamp);
            //     }
            // }
        }

            // if(cur.getMultiTagResult().estimatedPose.isPresent){
            //     Transform3d fieldToCamera = cur.getMultiTagResult().estimatedPose.best;

            //     Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

            //     var visionMeasure = camPose.transformBy(Constants.PhotonVisionConstants.CAMERA_TO_ROBOT);
            //     addVisionMeasurement(visionMeasure.toPose2d(), Units.millisecondsToSeconds((double)timestamp/1000));
            // }
            // else if(cur.hasTargets()){
            //     var timestamp = cur.metadata.captureTimestampMicros;
            //     var best = cur.getBestTarget();
            //     var bestID = best.getFiducialId();

            //     Optional<Pose3d> tagPose = layout.getTagPose(bestID);
            //     if(best.getPoseAmbiguity() <= 0.2 && bestID >= 0 && tagPose.isPresent()){
            //         var targetPose = tagPose.get();
            //         Transform3d camToTarget = best.getBestCameraToTarget();
            //         Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

            //         var visionMeasure = camPose.transformBy(Constants.PhotonVisionConstants.CAMERA_TO_ROBOT);
            //         addVisionMeasurement(visionMeasure.toPose2d(), Units.millisecondsToSeconds((double)timestamp/1000));
            //     }
            // }
        }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
    public SwerveRequest CameraGoToTag(double forward, double strafe, double turn, SwerveRequest.FieldCentric drive){     
        // TODO spend more time discussing other approaches to alignment code using odometry instead of raw vision data (Jerry + Ryan)           
        if(cur != null){
            if(cur.hasTargets()){
                PIDController pidControllerX = new PIDController(50, 0, 0.2); // TODO tune
                PIDController pidControllerY = new  PIDController(50, 0, 0.2); // TODO tune
                for(var target : cur.getTargets()) {
                    if (target.getFiducialId() == 7) {
                        // THIS SHOULD WORK FOR ALL APRILTAGS if bestCameraToTarget is robot relative
                        var xy = new Translation2d(target.bestCameraToTarget.getX(), target.bestCameraToTarget.getY());// assumes bestCameraToTarget is robot relative
                        var rotated = xy.rotateBy(Rotation2d.fromDegrees(360.0 -  getStateCopy().RawHeading.getDegrees()));

                        // forward = pidControllerX.calculate(rotated.getX(), Constants.PhotonVisionConstants.DES_HUMAN_DISTANCEX);
                        strafe = pidControllerY.calculate(rotated.getY(), Constants.PhotonVisionConstants.DES_HUMAN_DISTANCEY);
                        return new SwerveRequest.FieldCentricFacingAngle()
                            .withVelocityX(forward) // for tag 7, robot forward = negative x
                            .withVelocityY(strafe) // For tag 7, robot right = positive y
                            .withTargetDirection(Rotation2d.fromDegrees(target.getYaw() + 180.0));
                    }

                    // // If the camera to target is field relative
                    // // The following works for all tags
                    // if(target.getFiducialId() == 7) {
                    //     double curDistX = target.bestCameraToTarget.getX(); // assumes bestCameraToTarget is field relative
                    //     double curDistY = target.bestCameraToTarget.getY(); // assumes bestCameraToTarget is field relative

                    //     forward = pidControllerX.calculate(curDistX, Constants.PhotonVisionConstants.DES_HUMAN_DISTANCEX);
                    //     strafe = pidControllerY.calculate(curDistY, Constants.PhotonVisionConstants.DES_HUMAN_DISTANCEY);
                    //     return new SwerveRequest.FieldCentricFacingAngle()
                    //         .withVelocityX(forward) // for tag 7, robot forward = negative x
                    //         .withVelocityY(strafe) // For tag 7, robot right = positive y
                    //         .withTargetDirection(Rotation2d.fromDegrees(180));
                    // }
                }
            }
        }

        return drive.withVelocityX(forward)
                    .withVelocityY(strafe)
                    .withRotationalRate(turn);
    }

    public PhotonPipelineResult getPhotonResult() {
        return cur;
    }
    public void swerveDrive(double forward, double strafe, double angle){
        
    }
}