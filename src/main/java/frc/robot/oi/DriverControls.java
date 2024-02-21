package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControls {
  public double getDriveForward();

  public double getDriveLeft();

  public double getDriveRotation();

  public Trigger resetFieldCentric();
  

  // TESTING
  public Trigger setShooter45();

  public Trigger setClimberServoClose();

  public Trigger setClimberServoOpen();

  public default Trigger setClimberServoMove(){
    return new Trigger(()->false);

  }

  public double intakeNote();


 

}
