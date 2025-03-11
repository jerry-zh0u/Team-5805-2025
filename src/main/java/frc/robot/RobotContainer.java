// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.commands.AutoCoralManipulator;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.CoralManipulatorGo;
// import frc.robot.commands.ElevatorCommands;
// import frc.robot.commands.ZeroElevator;
import frc.robot.commands.GoToPoseCommand;
import frc.robot.commands.HuenemeElevatorZeroCmd;
import frc.robot.commands.WaitCommand;
import frc.robot.commands.ZeroElevator;
// import frc.robot.commands.ZeroElevator;
import frc.robot.constants.ElevatorHeights;
import frc.robot.subsystems.*;
// import frc.robot.subsystems.confs.CompbotElevatorConf;
import tagalong.TagalongConfiguration;
import tagalong.commands.base.ElevateToCmd;
import tagalong.commands.base.ElevatorZeroCmd;
import tagalong.subsystems.micro.Elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;

public class RobotContainer {
        public final TagalongElevatorSystem _elevator = new TagalongElevatorSystem(Constants.curRobot.elevatorConf);
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        // public final Elevator _elevator = new
        // Elevator(Constants.curRobot.elevatorConf);

        // private final CommandPS4Controller driver = new CommandPS4Controller(0);
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        // public final ElevatorSubsystem elevator = new ElevatorSubsystem();
        // public final ElevatorSubsystem elevator = new ElevatorSubsystem();

        public final CoralManipulator coralManip = new CoralManipulator();

        public final Climb climb = new Climb();

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                // _elevator.getPrimaryMotor().setPo;

                // Constants.curRobot.elevatorConf.elevatorConf.prim
                // _elevator.

                // NamedCommands.registerCommand("BASE", new SequentialCommasndGroup(
                // new ElevateToCmd(_elevator, ElevatorHeights.),
                // new ZeroElevator(elevator)));

                // NamedCommands.registerCommand("AUTOALIGN",
                // new GoToPoseCommand(drivetrain,
                // Constants.PhotonVisionConstants.DES_HUMAN_DISTANCE_Y_RIGHT)
                // .withTimeout(5));

                NamedCommands.registerCommand("BASE", _elevator.moveWithZero());
                NamedCommands.registerCommand("L1", new ElevateToCmd(_elevator,
                                ElevatorHeights.REEF_L1));
                NamedCommands.registerCommand("L2", new ElevateToCmd(_elevator,
                                ElevatorHeights.REEF_L2));
                NamedCommands.registerCommand("L3", new ElevateToCmd(_elevator,
                                ElevatorHeights.REEF_L3));
                NamedCommands.registerCommand("L4", new ElevateToCmd(_elevator,
                                ElevatorHeights.REEF_L4));
                // NamedCommands.registerCommand("L3", new ElevatorCommands(elevator,
                // Constants.L3));
                // NamedCommands.registerCommand("L4", new ElevatorCommands(elevator,
                // Constants.L4));
                NamedCommands.registerCommand("EJECT", new SequentialCommandGroup(
                                new CoralManipulatorGo(coralManip, -1),
                                new WaitCommand(0.25),
                                new CoralManipulatorGo(coralManip, 0)));
                // NamedCommands.registerCommand("EJECT", new AutoCoralManipulator(coralManip,
                // -1));

                // TagalongConfiguration.ffTuningMicrosystems.add("_elevator");
                // _elevator.get

                // DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
                // DogLog.setPdh(new PowerDistribution());

                // DogLog.log("ExampleLog", "Hello World");

                configureBindings();

                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("Auto Chooser", autoChooser);

                // coralManip.setDefaultCommand(new CoralManipulatorGo(coralManip, 0));
                // System.err.println("=====");
        }

        private void configureBindings() {

                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-Constants.driver.getLeftY() * MaxSpeed) // Drive
                                                                                                        // forward
                                                                                                        // with
                                                                                                        // negative
                                                                                                        // Y
                                                                                                        // (forward)4
                                                .withVelocityY(-Constants.driver.getLeftX() * MaxSpeed) // Drive
                                                                                                        // left
                                                                                                        // with
                                                                                                        // negative
                                                                                                        // X
                                                                                                        // (left)
                                                .withRotationalRate(
                                                                -Constants.driver.getRightX() * MaxAngularRate) // Drive
                                                                                                                // counterclockwise
                                                                                                                // with
                                                                                                                // negative
                                                                                                                // X
                                                                                                                // (left)
                                ));

                // driver.rightBumper().whileTrue(drivetrain.applyRequest(() ->
                // drivetrain.CameraGoToTag(-driver.getLeftY() * MaxSpeed, -driver.getLeftX() *
                // MaxSpeed, -driver.getRightX() * MaxAngularRate, drive)));
                // driver.L2().toggleOnTrue(new GoToPoseCommand(drivetrain, 7));
                // Constants.driver.L2().whileTrue(new GoToPoseCommand(drivetrain, 8));
                // Constants.driver.L1().onTrue(new SequentialCommandGroup(new
                // ElevatorCommands(elevator, 25), new WaitCommand(0.2), new
                // ElevatorCommands(elevator, Constants.ElevatorConstants.ELEVATORBASEHEIGHT)));
                // Constants.driver.R1().
                // Constants.driver.L1().onTrue(new ElevatorCommands(elevator, 25).andThen(new
                // WaitCommand(0.2).andThen(new ElevatorCommands(elevator,
                // Constants.ElevatorConstants.ELEVATORBASEHEI+GHT))));
                // Constants.driver.L1().onTrue(new ZeroElevator(elevator));
                Constants.driver.povRight()
                                .onTrue(new GoToPoseCommand(drivetrain,
                                                Constants.PhotonVisionConstants.DES_HUMAN_DISTANCE_Y_LEFT)
                                                .withTimeout(2));
                Constants.driver.povLeft()
                                .onTrue(new GoToPoseCommand(drivetrain,
                                                Constants.PhotonVisionConstants.DES_HUMAN_DISTANCE_Y_RIGHT)
                                                .withTimeout(2));

                Constants.driver.square().onTrue(new ConditionalCommand(
                                new ElevateToCmd(_elevator, ElevatorHeights.REEF_L1, true, 1.5),
                                new ElevateToCmd(_elevator, ElevatorHeights.REEF_L1, true, 0.5),
                                () -> _elevator.getElevator().getElevatorHeightM() < ElevatorHeights.REEF_L1
                                                .getHeightM()));

                Constants.driver.cross().onTrue(new ConditionalCommand(
                                new ElevateToCmd(_elevator, ElevatorHeights.REEF_L2, true, 1.5),
                                new ElevateToCmd(_elevator, ElevatorHeights.REEF_L2, true, 0.5),
                                () -> _elevator.getElevator().getElevatorHeightM() < ElevatorHeights.REEF_L2
                                                .getHeightM()));

                Constants.driver.circle().onTrue(new ConditionalCommand(
                                new ElevateToCmd(_elevator, ElevatorHeights.REEF_L3, true, 1.5),
                                new ElevateToCmd(_elevator, ElevatorHeights.REEF_L3, true, 0.5),
                                () -> _elevator.getElevator().getElevatorHeightM() < ElevatorHeights.REEF_L3
                                                .getHeightM()));

                Constants.driver.triangle().onTrue(new ConditionalCommand(
                                new ElevateToCmd(_elevator, ElevatorHeights.REEF_L4, true, 1.5),
                                new ElevateToCmd(_elevator, ElevatorHeights.REEF_L4, true, 0.5),
                                () -> _elevator.getElevator().getElevatorHeightM() < ElevatorHeights.REEF_L4
                                                .getHeightM()));

                Constants.driver.L1().onTrue(_elevator.moveWithZero());
                // Commands.sequence(
                // new ElevateToCmd<>(_elevator,
                // ElevatorHeights.BASE, true, .5)));

                // Constants.driver.L1().onTrue(new SequentialCommandGroup(
                // new ElevateToCmd(_elevator, ElevatorHeights.REEF_L1, false, 0.5),
                // new WaitCommand(1),
                // new ZeroElevator(_elevator)));
                /*
                 * Constants.driver.square().onTrue(new ElevateToCmd(_elevator,
                 * ElevatorHeights.REEF_L1, true));
                 * Constants.driver.cross().onTrue(new ElevateToCmd(_elevator,
                 * ElevatorHeights.REEF_L2, true));
                 * Constants.driver.circle().onTrue(new ElevateToCmd(_elevator,
                 * ElevatorHeights.REEF_L3, true));
                 * Constants.driver.triangle().onTrue(new ElevateToCmd(_elevator,
                 * ElevatorHeights.REEF_L4, true));
                 * 
                 * Constants.driver.L1().onTrue(new ElevateToCmd(_elevator,
                 * ElevatorHeights.BASE, true));
                 */
                // Constants.driver.L1().onTrue(new ElevateToCmd(_elevator,
                // ElevatorHeights.BASE, true, .5));

                // Constants.driver.L1().onTrue(new ElevatorZeroCmd(_elevator));
                // Constants.driver.L2().onTrue(new InstantCommand(() ->
                // _elevator.configMotor()));

                // Constants.driver.L1().onTrue(new SequentialCommandGroup(
                // new ElevatorCommands(elevator, Constants.L1),
                // new
                // ZeroElevator(elevator)).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf));

                // Constants.driver.square().onTrue(new ElevatorComands(elevator,
                // Constants.L1).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf));

                // Constants.driver.square().onTrue(new Elev)
                // Constants.driver.L1().onTrue(new ElevatorCommands(elevator, Constants.L1));
                // Constants.driver.cross().onTrue(new ElevatorCommands(elevator, Constants.L2)
                // .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf));
                // Constants.driver.circle().onTrue(new ElevatorCommands(elevator, Constants.L3)
                // .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf));
                // Constants.driver.triangle().onTrue(new ElevatorCommands(elevator,
                // Constants.L4)
                // .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf));
                // Constants.driver.povDown().onTrue(ElevatorCommands.zeroSubsystems());
                // Constants.driver.povUp().onTrue(new InstantCommand(() ->
                // ZeroElevator.stopZero()));
                // driver.L2().onTrue(Commands.runOnce(() -> elevator.zero(), elevator));
                // // driver.square().onTrue(new ElevatorCommands(elevator, 31.5));
                // // driver.circle().onTrue(new ElevatorCommands(elevator, 31.5));
                // driver.circle().onTrue(new ElevatorCommands(elevator, Constants.L1));
                // driver.cross().onTrue(new ElevatorCommands(elevator, Constants.L2));
                // driver.square().onTrue(new ElevatorCommands(elevator, Constants.L3));
                // driver.L1().onTrue(new ElevatorCommands(elevator, Constants.L4));

                Constants.operator.y().whileTrue(new ClimbCommands(climb, 1));
                Constants.operator.y().whileFalse(new ClimbCommands(climb, 0));
                Constants.operator.a().whileTrue(new ClimbCommands(climb, -1));
                Constants.operator.a().whileFalse(new ClimbCommands(climb, 0));

                // Constants.operator.rightBumper()

                Constants.driver.R1().whileTrue(new CoralManipulatorGo(coralManip, 0.8));
                Constants.driver.R1().whileFalse(new CoralManipulatorGo(coralManip, 0));
                Constants.driver.R2().whileTrue(new CoralManipulatorGo(coralManip, -0.8));
                Constants.driver.R2().whileFalse(new CoralManipulatorGo(coralManip, 0));

                // Constants.operator.leftBumper()
                // .onTrue(new SequentialCommandGroup(new ElevatorCommands(elevator,
                // elevator.getHeight() - 2),
                // new ElevatorCommands(elevator, elevator.getHeight() + 4),
                // new ElevatorCommands(elevator, MaxAngularRate)));

                // driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
                // driver.b().whileTrue(drivetrain.applyRequest(() ->
                // point.withModuleDirection(new Rotation2d(-driver.getLeftY(),
                // -driver.getLeftX()))
                // ));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // reset the field-centric heading on left bumper press
                Constants.operator.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                // Constants.operator.leftTrigger().onTrue(new
                // HuenemeElevatorZeroCmd(_elevator));

                // driver.cross().onTrue(new ElevateToCmd(_elevator, ElevatorHeights.REEF_L1));

                // driver.cross().onTrue(new ElevateToCmd(_elevator, ElevatorHeights.REEF_L1));
                // sequential command group OR andThen()
                // parallel command group OR alongWith()
                // parallel race group OR raceWith
                // parallel deadline group OR deadlineWith()

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public Command getAutonomousCommand() {
                // return autoChooser.getSelected();
                SmartDashboard.putString("Auto Selected", autoChooser.getSelected().getName());
                // System.err.println(autoChooser.getSelected().getName());
                return autoChooser.getSelected();
                // return new PathPlannerAuto("Test");
                // return Commands.print("No autonomous command configured");
        }

        public void onEnable() {
                _elevator.onEnable();
        }

        public void onDisable() {
                _elevator.onDisable();
        }
}
