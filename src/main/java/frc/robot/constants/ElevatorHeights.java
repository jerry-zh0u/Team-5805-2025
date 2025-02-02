package frc.robot.constants;

import tagalong.measurements.Height;

public enum ElevatorHeights implements Height {
  REEF_L1(0.4572),
  REEF_L2(0.809625),
  REEF_L3(1.209675),
  REEF_L4(1.8288),
  BASE(0),
  SCORE_BARGE(1.0), // TODO
  ELEVATOR_POST_SHOOT_LEAVE(1.0), // TODO
  GROUND_INTAKE_ALGAE(0.0), // TODO
  GROUND_INTAKE_CORAL(0.0), // TODO
  STATION_INTAKE_CORAL(0.0); // TODO

  private final double meters;

  ElevatorHeights(double meters) {
    this.meters = meters;
  }

  public double getHeightM() {
    return meters;
  }
}