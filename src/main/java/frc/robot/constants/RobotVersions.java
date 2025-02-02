package frc.robot.constants;

import frc.robot.subsystems.confs.CompbotElevatorConf;

public enum RobotVersions {
  COMPBOT(CompbotElevatorConf.construct());

  public final CompbotElevatorConf elevatorConf;
  RobotVersions(CompbotElevatorConf elevatorConf) {
    this.elevatorConf = elevatorConf;
  }
}
