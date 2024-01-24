package frc.robot.oi;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.utils.EricNubControls;


public class DriverControlsDualFlightStick implements DriverControls {
  public CommandJoystick m_leftJoystick;
  public CommandJoystick m_rightJoystick;
  public double m_deadzone;
  public EricNubControls m_controls;

  public DriverControlsDualFlightStick(int leftJoystick, int rightJoystick, double deadzone) {
    m_leftJoystick = new CommandJoystick(leftJoystick);
    m_rightJoystick = new CommandJoystick(rightJoystick);
    m_deadzone = deadzone;
    m_controls = new EricNubControls();
  }

  @Override
  public double getDriveForward() {
    double val = m_controls.addDeadzoneScaled(m_leftJoystick.getY(), m_deadzone);
    return -Math.signum(val) * Math.pow(val, 2);
  }

  @Override
  public double getDriveLeft() {
    double val = m_controls.addDeadzoneScaled(m_leftJoystick.getX(), m_deadzone);
    return -Math.signum(val) * Math.pow(val, 2);
  }

  @Override
  public double getDriveRotation() {
    double val = m_controls.addDeadzoneScaled(m_rightJoystick.getX(), m_deadzone);
    return -Math.signum(val) * Math.pow(val, 2);
  }

  @Override
  public Trigger setShooter45() {
    return m_rightJoystick.button(1);
  }

}
