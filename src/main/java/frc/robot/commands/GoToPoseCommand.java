package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class GoToPoseCommand extends Command  {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController pidControllerX = new PIDController(.05, 0, 0); // TODO tune
    private final PIDController pidControllerY = new  PIDController(.5, 0, 0); // TODO tune
    // private final PIDController pidControllerZ = new PIDController(.2, 0, 0);
    // private final PIDController pidControllerZ = new PIDController(50, 0, 0.2);
    private PhotonPipelineResult cur = new PhotonPipelineResult();

    private final int tagID;

    public GoToPoseCommand(CommandSwerveDrivetrain drivetrain, int tagID) {
        this.drivetrain = drivetrain;
        this.tagID = tagID;
    }

    @Override
    public void initialize() {
        pidControllerX.reset();
        pidControllerY.reset();

        pidControllerX.setTolerance(0.2);//FIXME CHECK TO SEE IF VALID
        pidControllerY.setTolerance(0.2);
    }

    @Override
    public void execute() {
        System.err.println("Pose Called");
        cur = drivetrain.getPhotonResult();
        // System.err.println(cur);
        if(cur != null){
            // System.err.println("++++");
            if(cur.hasTargets()){
                for(var target : cur.getTargets()) {
                    // if(target.getFiducialId() == 7){
                    //     System.err.println("Pose Found");
                    //     // double targetYaw = target.getYaw();
                    //     // double turn = -1.0 * targetYaw * 0.1 * 50;
                    //     // drivetrain.setControl(new);
                    //     drivetrain.setControl(new SwerveRequest.FieldCentricFacingAngle()
                    //         .withVelocityX(0.01) // for tag 7, robot forward = negative x
                    //         .withVelocityY(0.01) // For tag 7, robot right = positive y
                    //         .withTargetDirection(new Rotation2d(50)));
                    // }
                    // if(target.getFiducialId() == 7){
                    //     double targetYaw = target.getYaw();
                    //     double forward =
                    //             PhotonUtils.calculateDistanceToTargetMeters(
                    //                     .089, // Measured with a tape measure, or in CAD.
                    //                     .31, // From 2024 game manual for ID 7
                    //                     Units.degreesToRadians(15), // Measured with a protractor, or in CAD.
                    //                     Units.degreesToRadians(target.getPitch()));
                        
                    //     drivetrain.setControl(new SwerveRequest.FieldCentricFacingAngle()
                    //         .withVelocityX(forward)
                    //         .withVelocityY(strafe));
                    // }
                    // if (target.getFiducialId() == tagID) {
                    //     // System.err.println("=====");
                    //     // THIS SHOULD WORK FOR ALL APRILTAGS if bestCameraToTarget is robot relative
                    //     var xy = new Translation2d(target.bestCameraToTarget.getX(), target.bestCameraToTarget.getY());// assumes bestCameraToTarget is robot relative
                    //     var rotated = xy.rotateBy(Rotation2d.fromDegrees(360.0 -  drivetrain.getStateCopy().RawHeading.getDegrees()));
 
                    //     double forward = pidControllerX.calculate(rotated.getX(), Constants.PhotonVisionConstants.DES_HUMAN_DISTANCEX);
                    //     double strafe = pidControllerY.calculate(rotated.getY(), Constants.PhotonVisionConstants.DES_HUMAN_DISTANCEY);

                    //     System.err.println(forward + " "  + strafe);

                    //     drivetrain.setControl(new SwerveRequest.FieldCentric()
                    //         .withVelocityX(forward) // for tag 7, robot forward = negative x
                    //         .withVelocityY(strafe) // For tag 7, robot right = positive y
                    //         .withRotationalRate((target.getYaw()/* + 180.0*/) * Math.PI/180 ));
                    // }

                    // If the camera to target is field relative
                    // The following works for all tags
                    if(target.getFiducialId() == tagID) {
                        double curDistX = target.bestCameraToTarget.getX();
                        double curDistY = target.getYaw();

                        double forward = pidControllerX.calculate(curDistX, Constants.PhotonVisionConstants.DES_HUMAN_DISTANCEX);
                        double strafe = pidControllerY.calculate(curDistY, Constants.PhotonVisionConstants.DES_HUMAN_DISTANCEY);

                        // double curDistX = target.bestCameraToTarget.getX(); // assumes bestCameraToTarget is field relative
                        // double curDistY = target.bestCameraToTarget.getY(); // assumes bestCameraToTarget is field relative
                        // double curDistZ = target.bestCameraToTarget.getZ();
                        

                        // double forward = pidControllerX.calculate(curDistX, Constants.PhotonVisionConstants.DES_HUMAN_DISTANCEX);
                        // double strafe = pidControllerY.calculate(curDistY, Constants.PhotonVisionConstants.DES_HUMAN_DISTANCEY);
                        // double rotate = pidControllerZ.calculate(curDistZ, 180);

                        // System.err.println(curDistX + " " + curDistY + " " + curDistZ + " " + forward + " " + strafe + " " + rotate+ " ====");
                        // // double rotate = pidControllerZ.calculate(curDistZ, 1)

                        drivetrain.setControl(new SwerveRequest.FieldCentric()
                            .withVelocityX(forward) // for tag 7, robot forward = negative x
                            .withVelocityY(strafe) // For tag 7, robot right = positive y
                            .withRotationalRate(0));
                    }
                }
            }
        }
    }

    // @Override
    // public void end(boolean interrupted) {
    //     drivetrain.setControl(new SwerveRequest.FieldCentric().withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    // }

    @Override
    public boolean isFinished() {
        // return true if we're suitably close to the target
        // false otherwise
        // if(/*suitably close to target*/)
        //     return true;
        // if(pidControllerX.atSetpoint() && pidControllerY.atSetpoint()){
        //     return true;
        // }
        if(pidControllerY.atSetpoint() || cur.getTargets().isEmpty()){
            return true;
        }
        return false;
    }


    // public SwerveRequest CameraGoToTag(double forward, double strafe, double turn, SwerveRequest.FieldCentric drive){   
    //     // TODO spend more time discussing other approaches to alignment code using odometry instead of raw vision data (Jerry + Ryan)           
    //     if(cur != null){
    //         if(cur.hasTargets()){
    //             for(var target : cur.getTargets()) {
    //                 if (target.getFiducialId() == 7) {
    //                     // THIS SHOULD WORK FOR ALL APRILTAGS if bestCameraToTarget is robot relative
    //                     var xy = new Translation2d(target.bestCameraToTarget.getX(), target.bestCameraToTarget.getY());// assumes bestCameraToTarget is robot relative
    //                     var rotated = xy.rotateBy(Rotation2d.fromDegrees(360.0 -  getStateCopy().RawHeading.getDegrees()));

    //                     forward = pidControllerX.calculate(rotated.getX(), Constants.PhotonVisionConstants.DES_HUMAN_DISTANCEX);
    //                     strafe = pidControllerY.calculate(rotated.getY(), Constants.PhotonVisionConstants.DES_HUMAN_DISTANCEY);
    //                     return new SwerveRequest.FieldCentricFacingAngle()
    //                         .withVelocityX(forward) // for tag 7, robot forward = negative x
    //                         .withVelocityY(strafe) // For tag 7, robot right = positive y
    //                         .withTargetDirection(Rotation2d.fromDegrees(target.getYaw() + 180.0));
    //                 }

    //                 // // If the camera to target is field relative
    //                 // // The following works for all tags
    //                 // if(target.getFiducialId() == 7) {
    //                 //     double curDistX = target.bestCameraToTarget.getX(); // assumes bestCameraToTarget is field relative
    //                 //     double curDistY = target.bestCameraToTarget.getY(); // assumes bestCameraToTarget is field relative

    //                 //     forward = pidControllerX.calculate(curDistX, Constants.PhotonVisionConstants.DES_HUMAN_DISTANCEX);
    //                 //     strafe = pidControllerY.calculate(curDistY, Constants.PhotonVisionConstants.DES_HUMAN_DISTANCEY);
    //                 //     return new SwerveRequest.FieldCentricFacingAngle()
    //                 //         .withVelocityX(forward) // for tag 7, robot forward = negative x
    //                 //         .withVelocityY(strafe) // For tag 7, robot right = positive y
    //                 //         .withTargetDirection(Rotation2d.fromDegrees(180));
    //                 // }
    //             }
    //         }
    //     }

    //     return drive.withVelocityX(forward)
    //                 .withVelocityY(strafe)
    //                 .withRotationalRate(turn);
    // }   
}
