package frc.robot.constants;

import frc.robot.subsystems.confs.TagalongElevatorSubsystemConf;

public enum RobotVersions {
  COMPBOT(TagalongElevatorSubsystemConf.construct());

  public final TagalongElevatorSubsystemConf elevatorConf;

  RobotVersions(TagalongElevatorSubsystemConf elevatorConf) {
    this.elevatorConf = elevatorConf;
  }
}
