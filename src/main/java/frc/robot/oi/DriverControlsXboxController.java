package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.utils.EricNubControls;

public class DriverControlsXboxController implements DriverControls {

  CommandXboxController m_controller;
  EricNubControls m_controls;

  public DriverControlsXboxController(int xboxControllerPort) {
    m_controller = new CommandXboxController(xboxControllerPort);
    m_controls = new EricNubControls();
  }

  @Override
  public double getDriveForward() {
    double val = m_controls.addDeadzoneScaled(m_controller.getLeftY(), 0.03);
    return -Math.signum(val) * Math.pow(val, 2);
  }

  @Override
  public double getDriveLeft() {
    double val = m_controls.addDeadzoneScaled(m_controller.getLeftX(), 0.03);
    return -Math.signum(val) * Math.pow(val, 2);
  }

  @Override
  public double getDriveRotation() {
    double val = m_controls.addDeadzoneScaled(m_controller.getRightX(), 0.03);
    return -Math.signum(val) * Math.pow(val, 2);
  }
  @Override
  public Trigger setShooter45() {
    return m_controller.button(1);
  }

}
