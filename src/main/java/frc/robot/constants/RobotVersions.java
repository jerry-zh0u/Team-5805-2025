package frc.robot.constants;

import frc.robot.subsystems.confs.compbotElevatorConf;
import frc.robot.subsystems.confs.ElevatorConf;
public enum RobotVersions {
  COMPBOT(compbotElevatorConf.construct());
  public final ElevatorConf elevatorConf;
  RobotVersions(ElevatorConf elevatorConf) {
    this.elevatorConf = elevatorConf;
  }
}
