package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Filesystem;

import tagalong.subsystems.micro.Elevator;
import tagalong.subsystems.micro.augments.ElevatorAugment;
import frc.robot.subsystems.confs.ElevatorConf;
import tagalong.subsystems.TagalongSubsystemBase;

public class Elevator extends TagalongSubsystemBase implements ElevatorAugment {
  public static final int Elevator_ID = 0;

  private final Elevator _elevator;

  public final ElevatorConf _elevatorConf;

//   /* -------- Logging: utilities and configs -------- */
//   private final ElevatorIOTalonFX _io;
//   private final ElevatorIOInputsAutoLogged _inputs = new ElevatorIOInputsAutoLogged();

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
  public Elevator(ElevatorConf elevatorConf) {
    super(elevatorConf);
    _elevator = new Elevator( elevatorConf!= null ? elevatorConf.elevatorConf : null);
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
}

  @Override 
  public void onEnable () {
   if (_isSubsystemDisabled) { 
  return;
    }
      _elevator.onEnable();
  } 

  @Override 
  public void onDisable () {
   if (_isSubsystemDisabled) { 
  return;
    }
      _elevator.onDisable();
  } 

  @Override 
  public void periodic () {
   if (_isSubsystemDisabled) { 
  return;
    }
      _elevator.periodic();
    updateShuffleboard();
  } 

 public void simulationInit (){
      _elevator.simulationInit();
  } 

 public void simulationPeriodic (){
      _elevator.simulationPeriodic();
  } 

 public void updateShuffleboard (){
      _elevator.updateShuffleboard();
  } 

 public void configShuffleboard (){
      _elevator.configShuffleboard();
  } 

public boolean checkInitStatus() {
  return  _elevator.checkInitStatus();} 

}
