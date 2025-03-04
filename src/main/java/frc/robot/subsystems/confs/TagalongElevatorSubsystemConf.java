package frc.robot.subsystems.confs;

import tagalong.subsystems.micro.confs.ElevatorConf;
import frc.robot.subsystems.confs.micro.CompbotElevatorElevatorConf;

public class TagalongElevatorSubsystemConf extends ElevatorSystemConf {
  public static final ElevatorConf elevatorConf = CompbotElevatorElevatorConf.construct();

  public static TagalongElevatorSubsystemConf construct() {
    return new TagalongElevatorSubsystemConf(elevatorConf);
  }

  public TagalongElevatorSubsystemConf(ElevatorConf elevatorConf) {
    super(elevatorConf);
  }

}
