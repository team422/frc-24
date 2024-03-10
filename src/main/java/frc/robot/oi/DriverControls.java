package frc.robot.oi;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControls {

  
  public double getDriveForward();

  public double getDriveLeft();

  public double getDriveRotation();

  public Trigger resetFieldCentric();
  



  public Trigger setClimberServoClose();

  public Trigger setClimberServoOpen();




  public Trigger goToIntakePosition();

  public default Trigger goToShootPositionAndRev(){
    return new Trigger(()->false);
  }

  public default Trigger finalShoot(){
    return new Trigger(()->false);
  }

  public default Trigger ampAutoLineup(){
    return new Trigger(()->false);
  }

  public default void setAmpAutoLineup(boolean value){
  }

  public default void setDriverRumble(double value, RumbleType side){
  }

   public default Trigger fenderShot(){return new Trigger(()->false);};
  

   public default Trigger autoIntake() {
    return new Trigger(()->false);
   }

   public default Trigger intakeVomit(){
    return new Trigger(()->false);
   }

   public default Trigger runTune(){
    return new Trigger(()->false);
   }

   public default Trigger hockeyPuck(){
    return new Trigger(()->false);
   }

   public default Trigger autoAlignToGamePiece(){
    return new Trigger(()->false);
   }

  public default Trigger testRumble() {
    return new Trigger(() -> false);
  }

 

}
