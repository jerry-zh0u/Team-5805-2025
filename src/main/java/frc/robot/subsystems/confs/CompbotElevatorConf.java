package frc.robot.subsystems.confs;

import tagalong.subsystems.micro.confs.ElevatorConf;
import frc.robot.subsystems.confs.micro.CompbotElevatorElevatorConf;

public class CompbotElevatorConf extends ElevatorConf {
    public static final ElevatorConf elevatorConf = CompbotElevatorElevatorConf.construct();

    public static CompbotElevatorConf construct() {
        return new CompbotElevatorConf (elevatorConf);
    }
    public CompbotElevatorConf(ElevatorConf elevatorConf) {
super(    elevatorConf);
  }
}
