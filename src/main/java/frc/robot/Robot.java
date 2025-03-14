// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import tagalong.TagalongConfiguration;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    TagalongConfiguration.ffTuningMicrosystems.add("CoralElevator");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Elevator Height in Meters",
        m_robotContainer._elevator.getElevator().getElevatorHeightM());
  }

  @Override
  public void disabledInit() {
    // m_robotContainer.elevator.setMode(true);
    m_robotContainer.onDisable();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
    // m_robotContainer.elevator.setMode(false);
    // m_robotContainer.elevator.zero();
    // m_robotContainer._elevator.getElevator().
    // m_robotContainer._elevator.getElevator().getPrimaryMotor().setPosition(0);
    m_robotContainer.climb.setZero();

    // m_robotContainer._elevator.setPower();// v_min = 0.015 v_max = .04

    // m_robotContainer._elevator.setPower(0.018);

    // m_robotContainer._elevator.setPower(0.1);
    // m_robotContainer.elevator.setVoltage(.05);
    // m_robotContainer.elevator.setVoltage(-2);

    // m_robotContainer._elevator.getElevator().getPrimaryMotor().setControl(new
    // VelocityVoltage(0).withFeedForward(1.04));
    // -0.24, -0.47
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
