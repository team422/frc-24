package frc.robot.oi;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.utils.EricNubControls;

public class DriverControlsXboxController implements DriverControls {

  // CommandXboxController m_controller;
  CommandPS5Controller m_controller;
  EricNubControls m_controls;

  public DriverControlsXboxController(int xboxControllerPort) {
    m_controller = new CommandPS5Controller(xboxControllerPort);
    m_controls = new EricNubControls();
  }

  @Override
  public double getDriveForward() {
    double val = m_controls.addDeadzoneScaled(m_controller.getLeftY(), 0.1);
    return -Math.signum(val) * Math.pow(val, 2);
  }

  @Override
  public double getDriveLeft() {
    double val = m_controls.addDeadzoneScaled(m_controller.getLeftX(), 0.1);
    return -Math.signum(val) * Math.pow(val, 2);
  }

  @Override
  public double getDriveRotation() {
    double val = m_controls.addDeadzoneScaled(m_controller.getRightX(), 0.03);
    return -Math.signum(val) * Math.pow(val, 4);
  }

  @Override
  public Trigger resetFieldCentric(){
    return m_controller.povUp();
  }



  @Override
  public Trigger setClimberServoClose() {
    return m_controller.povRight();
  }

  @Override
  public Trigger setClimberServoOpen() {
    return m_controller.povLeft();
  }

  

  

  @Override
  public Trigger goToIntakePosition(){
    return new Trigger(()->m_controller.getR2Axis()>0.1);
  }

  @Override
  public Trigger goToShootPositionAndRev(){
    return m_controller.L2();
  }


  @Override
  public Trigger finalShoot() {
    return m_controller.R1();
  }

  @Override
  public Trigger fenderShot(){
    return m_controller.triangle();
  }

  @Override
  public Trigger ampAutoLineup(){
    return m_controller.L1();
  }

  @Override
  public void setDriverRumble(double rumble, RumbleType side){
    m_controller.getHID().setRumble(side, rumble);
  }

  @Override
  public Trigger autoIntake(){
    return m_controller.circle();
  }

  @Override
  public Trigger intakeVomit(){
    return m_controller.R3();
  }

  @Override
  public Trigger runTune(){
    return m_controller.square();
  }

  @Override
  public Trigger hockeyPuck(){
    return m_controller.cross();
  }







}
