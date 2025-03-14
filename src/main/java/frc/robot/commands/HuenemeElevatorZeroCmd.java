/**
 * Copyright 2024 The Space Cookies : Girl Scout Troop #62868 and FRC Team #1868
 * Open Source Software; you may modify and/or share it under the terms of
 * the 3-Clause BSD License found in the root directory of this project.
 */
package frc.robot.commands;

// package tagalong.commands.base;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import tagalong.commands.TagalongCommand;
import tagalong.subsystems.TagalongSubsystemBase;
import tagalong.subsystems.micro.Elevator;
import tagalong.subsystems.micro.augments.ElevatorAugment;

/**
 * Command that finds the elevator zero position and sets the encoder position
 * to zero.
 */
public class HuenemeElevatorZeroCmd<T extends TagalongSubsystemBase & ElevatorAugment>
    extends TagalongCommand {
  /**
   * Elevator subsystem.
   */
  private final Elevator _elevator;
  /**
   * The height of the elevator when initialized, in meters.
   */
  private double _prevHeightM;
  /**
   * Timer to track the stall duration to ensure a true bottom
   */
  private Timer stallTimer = new Timer();

  /**
   * Construct the command according to the below parameters.
   *
   * @param elevator the elevator subsystem
   */
  public HuenemeElevatorZeroCmd(T elevator) {
    _elevator = elevator.getElevator();
    addRequirements(elevator);
  }

  /**
   * Construct the command according to the below parameters.
   *
   * @param id       Integer ID of the elevator microsystem inside the Tagalong
   *                 Subsystem
   * @param elevator the elevator subsystem
   */
  public HuenemeElevatorZeroCmd(int id, T elevator) {
    _elevator = elevator.getElevator(id);
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    _elevator.setPrimaryPower(-0.05);
    _prevHeightM = _elevator.getElevatorHeightM();
    _elevator.setHoldPosition(false);
    stallTimer.reset();
  }

  @Override
  public void execute() {
    double currentHeightM = _elevator.getElevatorHeightM();
    if (Math.abs(_prevHeightM - currentHeightM) > 0.001) { // ~.04][\] inches
      stallTimer.reset();
    } else {
      stallTimer.start();
    }
    _prevHeightM = currentHeightM;
  }

  @Override
  public void end(boolean interrupted) {
    _elevator.setPrimaryPower(0.0);
    if (!interrupted)
      _elevator.setElevatorHeight(0.0);
  }

  @Override
  public boolean isFinished() {
    return stallTimer.hasElapsed(0.1);
  }
}
