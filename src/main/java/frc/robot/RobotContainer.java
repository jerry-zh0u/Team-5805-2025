// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.commands.CoralManipulatorGo;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.GoToPoseCommand;
import frc.robot.constants.ElevatorHeights;
import frc.robot.subsystems.*;
import tagalong.TagalongConfiguration;
import tagalong.commands.base.ElevateToCmd;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;

public class RobotContainer {
  // public final TaglalongElevatorSystem _elevator = new TaglalongElevatorSystem(Constants.curRobot.elevatorConf);
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandPS4Controller joystick = new CommandPS4Controller(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final ElevatorSubsystem elevator = new ElevatorSubsystem();

    public final CoralManipulator coralManip = new CoralManipulator();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();

        // NamedCommands.registerCommand("BASE", new ElevateToCmd(_elevator, ElevatorHeights.BASE));
        // NamedCommands.registerCommand("L1", new ElevateToCmd(_elevator, ElevatorHeights.REEF_L1));
        // NamedCommands.registerCommand("L2", new ElevateToCmd(_elevator, ElevatorHeights.REEF_L2));
        // NamedCommands.registerCommand("L3", new ElevateToCmd(_elevator, ElevatorHeights.REEF_L3));
        // NamedCommands.registerCommand("L4", new ElevateToCmd(_elevator, ElevatorHeights.REEF_L4));

        configureBindings();
        TagalongConfiguration.ffTuningMicrosystems.add("_elevator");
        // _elevator.get
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // coralManip.setDefaultCommand(new CoralManipulatorGo(coralManip, 0));
        // System.err.println("=====");
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)4
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> drivetrain.CameraGoToTag(-joystick.getLeftY() * MaxSpeed, -joystick.getLeftX() * MaxSpeed, -joystick.getRightX() * MaxAngularRate, drive)));
        // joystick.R1().whileTrue(new GoToPoseCommand(drivetrain, 7));
        // joystick.R1().whileTrue(new GoToPoseCommand(drivetrain, 7));

        // joystick.R1().
        // joystick.R1().onTrue(new ElevatorCommands(elevator, Constants.L0));
        // joystick.L1().onTrue(new ElevatorCommands(elevator, Constants.L4));
        joystick.triangle().onTrue(new ElevatorCommands(elevator, Constants.L0));
        joystick.circle().onTrue(new ElevatorCommands(elevator, Constants.L1));
        joystick.cross().onTrue(new ElevatorCommands(elevator, Constants.L2));
        joystick.square().onTrue(new ElevatorCommands(elevator, Constants.L3));
        joystick.L1().onTrue(new ElevatorCommands(elevator, Constants.L4));

        joystick.L2().onTrue(Commands.runOnce(() -> elevator.zero(), elevator));

        joystick.R1().whileTrue(new CoralManipulatorGo(coralManip, 1));
        joystick.R1().whileFalse(new CoralManipulatorGo(coralManip, 0));
        joystick.R2().whileTrue(new CoralManipulatorGo(coralManip, -1));
        joystick.R2().whileFalse(new CoralManipulatorGo(coralManip, 0));

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // joystick.cross().onTrue(new ElevateToCmd(_elevator, ElevatorHeights.REEF_L1));
        
        // joystick.cross().onTrue(new ElevateToCmd(_elevator, ElevatorHeights.REEF_L1));
        // sequential command group OR andThen()
        // parallel command group OR alongWith()
        // parallel race group OR raceWith()
        // parallel deadline group OR deadlineWith()
      
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        // return new PathPlannerAuto("B Auto 1");
        // return Commands.print("No autonomous command configured");
    }  
        
  //   public void onEnable() {
  //   _elevator.onEnable();
  // }
  // public void onDisable() {
  //   _elevator.onDisable();
  // }

//   @Override
//   public void teleopInit() {
//     if (m_autonomousCommand != null) {
//       m_autonomousCommand.cancel();
//     }
//   }
}

