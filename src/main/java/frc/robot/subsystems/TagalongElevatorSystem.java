package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import tagalong.subsystems.micro.Elevator;
import tagalong.subsystems.micro.augments.ElevatorAugment;
import frc.robot.Constants;
import frc.robot.constants.ElevatorHeights;
import frc.robot.subsystems.confs.ElevatorSystemConf;
import tagalong.subsystems.TagalongSubsystemBase;

public class TagalongElevatorSystem extends TagalongSubsystemBase implements ElevatorAugment {
  public static final int Elevator_ID = 0;

  private final Elevator _elevator;

  public final ElevatorSystemConf _elevatorConf;

  private DigitalInput limitSwitch;
  private boolean zeroed;

  // /* -------- Logging: utilities and configs -------- */
  // private final ElevatorIOTalonFX _io;
  // private final ElevatorIOInputsAutoLogged _inputs = new
  // ElevatorIOInputsAutoLogged();

  @Override
  public Elevator getElevator() {
    return _elevator;
  }

  @Override
  public Elevator getElevator(int id) {
    switch (id) {
      case 0:
      default:
        return _elevator;
    }
  }

  public TagalongElevatorSystem(ElevatorSystemConf elevatorConf) {
    super(elevatorConf);
    _elevator = new Elevator(elevatorConf != null ? elevatorConf.elevatorConf : null);

    // System.err.println(_elevator._elevatorZeroingPower);
    // _elevator._elevatorZeroingPower;
    if (_elevator._configuredMicrosystemDisable) {
      _elevatorConf = null;
      return;
    }
    _elevatorConf = elevatorConf;

    int counter = 0;
    while (!checkInitStatus() && counter < 100) {
      System.out.println("Waiting for Elevator");
    }

    if (counter >= 100) {
      System.out.println("failed to init Elevator");
    }

    configShuffleboard();

    // _elevator.getPrimaryMotor().setPosition(0);
    // _elevator._allMotors[1].setPosition(0 );
    limitSwitch = new DigitalInput(Constants.ElevatorConstants.LIMITSWITCHID);
    zeroed = true;
  }

  @Override
  public void onEnable() {
    if (_isSubsystemDisabled) {
      return;
    }
    _elevator.onEnable();
    // _elevator.getPrimaryMotor().setPosition(0);
  }

  @Override
  public void onDisable() {
    if (_isSubsystemDisabled) {
      return;
    }
    _elevator.onDisable();
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  @Override
  public void periodic() {
    // _elevator._elevatorZeroingPower = 5;
    SmartDashboard.putNumber("Elevator Zeroing Power", _elevator._elevatorZeroingPower);
    SmartDashboard.putNumber("Elevator Zeroing Time", _elevator._elevatorZeroingDurationS);

    SmartDashboard.putBoolean("Limit Switch", limitSwitch.get());

    // if (limitSwitch.get()) {
    // if (!zeroed) {
    // // _elevator.setElevatorProfile(ElevatorHeights.REEF_L2.getHeightM());
    // _elevator.getPrimaryMotor().setPosition(0);
    // _elevator.setPrimaryPower(0);
    // // _elevator.getPrimaryMotor().setVoltage(0);
    // zeroed = true;
    // }
    // } else {
    // zeroed = false;

    // }

    if (_isSubsystemDisabled) {
      return;
    }
    _elevator.periodic();
    updateShuffleboard();
  }

  public void setPower(double amt) {
    _elevator.setPrimaryPower(amt);
  }

  public void zero() {
    setPower(0);
    // _elevator.getPrimaryMotor().setPosition(0);
  }

  public void simulationInit() {
    _elevator.simulationInit();
  }

  public void simulationPeriodic() {
    _elevator.simulationPeriodic();
  }

  public void updateShuffleboard() {
    _elevator.updateShuffleboard();
  }

  public void configShuffleboard() {
    _elevator.configShuffleboard();
  }

  public boolean checkInitStatus() {
    return _elevator.checkInitStatus();
  }

}
