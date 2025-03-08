package frc.robot.subsystems.confs;

import tagalong.subsystems.micro.confs.ElevatorConf;

public class ElevatorSystemConf {
  public ElevatorConf elevatorConf;

  public ElevatorSystemConf(ElevatorConf elevatorConf) {
    this.elevatorConf = elevatorConf;
    // elevatorConf.getPrim
    // elevatorConf.elevatorZeroingStallToleranceM = 2;
  }
}
