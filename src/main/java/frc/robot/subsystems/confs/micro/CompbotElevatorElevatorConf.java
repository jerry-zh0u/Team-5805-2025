  package frc.robot.subsystems.confs.micro;

  import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
  import com.ctre.phoenix6.signals.InvertedValue;
  import com.ctre.phoenix6.signals.NeutralModeValue;
  import tagalong.devices.Motors;
  import tagalong.controls.FeedforwardConstants;
  import tagalong.controls.PIDSGVAConstants;
  import tagalong.units.AccelerationUnits;
  import tagalong.units.DistanceUnits;
  import tagalong.units.MassUnits;
  import tagalong.units.VelocityUnits;
  import tagalong.subsystems.micro.confs.ElevatorConf;


public class CompbotElevatorElevatorConf extends ElevatorConf {
   public static final String name = "Elevator";
  public static final Motors[] motorTypes = {Motors.KRAKEN_X60};
  public static final int[] motorDeviceIDs = {5, 6};
  public static final String[] motorCanBus = {"rio"};

  // public static final InvertedValue[] motorDirection = {InvertedValue.CounterClockwise_Positive};
  public static final InvertedValue[] motorDirection = {InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive};
  public static final NeutralModeValue[] motorEnabledBrakeMode = {NeutralModeValue.Coast, NeutralModeValue.Coast};
  public static final NeutralModeValue[] motorDisabledBrakeMode = {NeutralModeValue.Coast, NeutralModeValue.Coast};

  public static final int[][] gearRatio = {{5, 1}, {5, 1}};

  public static final boolean motorCurrentLimitStatorEnableLimit = true;
  public static final int motorCurrentLimitStatorPeakLimit = 80;
  public static final boolean motorCurrentLimitSupplyEnableLimit = true;
  public static final int motorCurrentLimitSupplyPeakLimit = 80;
  public static final int motorCurrentLimitSupplyContinuousLimit = 80;
  public static final double motorCurrentLimitPeakDuration = 0.8;

public static final FeedforwardConstants feedForward =
    new FeedforwardConstants(0.0, 0.0, 0.0, 0.0);

/* -------- Positional -------- */
public static final PIDSGVAConstants slot0 =
    new PIDSGVAConstants(0.001, 0.0, 0.0, 1, 0.0, 0.0, 0.0);
/* -------- Velocity -------- */
public static final PIDSGVAConstants slot1 =
    new PIDSGVAConstants(0.000, 0.0, 0.0, 1, 0.0, 0.0, 0.0);
/* -------- Current -------- */
public static final PIDSGVAConstants slot2 =
    new PIDSGVAConstants(0.000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

public static final boolean closedLoopConfigsContinuousWrap = true;

public static final DistanceUnits ffOffsetUnit = DistanceUnits.METER;
public static final double ffOffsetValue = 0.0;

public static final DistanceUnits profileOffsetUnit = DistanceUnits.METER;
public static final double profileOffsetValue = 0.0;

/* -------- Simulation Specific Control -------- */
public static final FeedforwardConstants simFeedForward =
    new FeedforwardConstants(0.0, 0.0, 0.0, 0.0);
/* -------- Positional -------- */
public static final PIDSGVAConstants simSlot0 =
    new PIDSGVAConstants(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
/* -------- Velocity -------- */
public static final PIDSGVAConstants simSlot1 =
    new PIDSGVAConstants(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
/* -------- Current -------- */
public static final PIDSGVAConstants simSlot2 =
    new PIDSGVAConstants(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

public static final MassUnits carriageMassUnit = MassUnits.KILOGRAMS;
public static final double carriageMassValue = 0.0;
public static final double mech2dDim = 1.0;
public static final String rootName = "Elevator Base";
public static final double rootX = 0.5;
public static final double rootY = 0.0;
public static final double lineLength = 0.3;
public static final double angle = 90;
public static final DistanceUnits drumDiameterUnits = DistanceUnits.METER;
public static final double drumDiameter = 0.0;

  /* ----- Required Tuning ------ */
  private static final DistanceUnits trapezoidalLimitsUnits = DistanceUnits.METER;
  private static final VelocityUnits trapezoidalVelocityUnits = VelocityUnits.METERS_PER_SECOND;
  private static final AccelerationUnits trapezoidalAccelerationUnits = AccelerationUnits.METERS_PER_SECOND2;
  private static final double trapezoidalLimitsVelocity = 0.01;
  private static final double trapezoidalLimitsAcceleration = 0.0;

  public static final DistanceUnits positionalLimitsUnits = DistanceUnits.METER;
  public static final double positionalLimitsMin = 0.0;
  public static final double positionalLimitsMax = 0.5;
  public static final DistanceUnits defaultTolerancesUnit = DistanceUnits.METER;
  public static final double defaultLowerTolerance = 0.0;
  public static final double defaultUpperTolerance = 1.0;

    public static CompbotElevatorElevatorConf
    construct() {
  // placeholder
  return new CompbotElevatorElevatorConf(
      name,
      motorTypes,
      motorDeviceIDs,
      motorCanBus,
      motorDirection,
      motorEnabledBrakeMode,
      motorDisabledBrakeMode,
      gearRatio,
      positionalLimitsUnits,
      positionalLimitsMin,
      positionalLimitsMax,
      trapezoidalLimitsUnits,
      trapezoidalVelocityUnits,
      trapezoidalLimitsVelocity,
      trapezoidalAccelerationUnits,
      trapezoidalLimitsAcceleration,
      defaultTolerancesUnit,
      defaultLowerTolerance,
      defaultUpperTolerance,
      feedForward,
      simFeedForward,
      new CurrentLimitsConfigs()
          .withStatorCurrentLimit(motorCurrentLimitStatorPeakLimit)
          .withSupplyCurrentLimit(motorCurrentLimitSupplyPeakLimit)
          .withSupplyCurrentLowerLimit(motorCurrentLimitSupplyContinuousLimit)
          .withSupplyCurrentLowerTime(motorCurrentLimitPeakDuration)
          .withStatorCurrentLimitEnable(motorCurrentLimitStatorEnableLimit)
          .withSupplyCurrentLimitEnable(motorCurrentLimitSupplyEnableLimit),
      slot0,
      slot1,
      slot2,
      carriageMassUnit,
      carriageMassValue,
      mech2dDim,
      rootName,
      rootX,
      rootY,
      lineLength,
      angle,
      simSlot0,
      simSlot1,
      simSlot2,
      drumDiameterUnits,
      drumDiameter
  );
}
public CompbotElevatorElevatorConf(
  String name,
  Motors[] motorTypes,
  int[] motorDeviceIDs,
  String[] motorCanBus,
  InvertedValue[] motorDirection,
  NeutralModeValue[] motorEnabledBrakeMode,
  NeutralModeValue[] motorDisabledBrakeMode,
  int[][] gearRatio,
  DistanceUnits positionalLimitsUnits,
  double positionalMin,
  double positionalMax,
  DistanceUnits trapezoidalLengthUnit,
  VelocityUnits trapezoidalVelocityUnit,
  double trapezoidalLimitsVelocity,
  AccelerationUnits trapezoidalAccelerationUnit,
  double trapezoidalLimitsAcceleration,
  DistanceUnits defaultTolerancesUnit,
  double defaultLowerTolerance,
  double defaultUpperTolerance,
  FeedforwardConstants feedForward,
  FeedforwardConstants simFeedForward,
  CurrentLimitsConfigs currentLimitsConfigs,
  PIDSGVAConstants slot0,
  PIDSGVAConstants slot1,
  PIDSGVAConstants slot2,
  MassUnits carriageMassUnit,
  double carriageMassValue,
  double mech2dDim,
  String rootName,
  double rootX,
  double rootY,
  double lineLength,
  double angle,
  PIDSGVAConstants simSlot0,
  PIDSGVAConstants simSlot1,
  PIDSGVAConstants simSlot2,
  DistanceUnits drumDiameterUnits,
  double drumDiameter) {
  super(
    name,
    motorTypes,
    motorDeviceIDs,
    motorCanBus,
    motorDirection,
    motorEnabledBrakeMode,
    motorDisabledBrakeMode,
    gearRatio,
    positionalLimitsUnits,
    positionalMin,
    positionalMax,
    trapezoidalLimitsUnits,
    trapezoidalVelocityUnits,
    trapezoidalLimitsVelocity,
    trapezoidalAccelerationUnits,
    trapezoidalLimitsAcceleration,
    defaultTolerancesUnit,
    defaultLowerTolerance,
    defaultUpperTolerance,
    feedForward,
    simFeedForward,
    new CurrentLimitsConfigs()
        .withStatorCurrentLimit(motorCurrentLimitStatorPeakLimit)
        .withSupplyCurrentLimit(motorCurrentLimitSupplyPeakLimit)
        .withSupplyCurrentLowerLimit(motorCurrentLimitSupplyContinuousLimit)
        .withSupplyCurrentLowerTime(motorCurrentLimitPeakDuration)
        .withStatorCurrentLimitEnable(motorCurrentLimitStatorEnableLimit)
        .withSupplyCurrentLimitEnable(motorCurrentLimitSupplyEnableLimit),
    slot0,
    slot1,
    slot2,
    carriageMassUnit,
    carriageMassValue,
    mech2dDim,
    rootName,
    rootX,
    rootY,
    lineLength,
    angle,
    simSlot0,
    simSlot1,
    simSlot2,
    drumDiameterUnits,
    drumDiameter);
  }

}