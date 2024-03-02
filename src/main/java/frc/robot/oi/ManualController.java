package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ManualController {
    CommandXboxController m_controller;
    public ManualController(int port) {
        m_controller = new CommandXboxController(port);
    }

    public double getShooterStick() {
        return m_controller.getLeftTriggerAxis();
    }

    public double getFeederStick() {
        return m_controller.getRightTriggerAxis();
    }

    public double climberStick() {
        return m_controller.getLeftY();
    }

    public Trigger cimberLockToggle() {
        return m_controller.a();
    }

    public Trigger intakeUpDegrees() {
        return m_controller.b();
    }

    public Trigger intakeDownDegrees() {
        return m_controller.x();
    }

    public Trigger finalShoot() {
        return m_controller.rightBumper();
    }

    public Trigger maxSpeedShooter() {
        return m_controller.y();
    }


    public Trigger amp(){
        return m_controller.b();
    }

}
