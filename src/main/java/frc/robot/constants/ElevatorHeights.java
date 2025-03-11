package frc.robot.constants;

import tagalong.measurements.Height;

public enum ElevatorHeights implements Height {
  REEF_L1(0.15),
  REEF_L2(0.212),
  REEF_L3(.405),
  REEF_L4(.72),
  BASE(0.04);

  private final double meters;

  ElevatorHeights(double meters) {
    this.meters = meters;
  }

  public double getHeightM() {
    return meters;
  }
}