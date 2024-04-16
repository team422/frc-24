package frc.robot.oi;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ManualController {
    CommandPS4Controller m_controller;
    public ManualController(int port) {
        m_controller = new CommandPS4Controller(port);
    }

    public double getShooterStick() {
        return 0.0;
    }

    public double getFeederStick() {
        return 0.0;
    }

    public double climberStick() {
        return m_controller.getLeftY();
    }

    public Trigger cimberLockToggle() {
        return new Trigger(()->false);
    }

    public Trigger intakeUpDegrees() {
        return new Trigger(()->false);
    }

    public Trigger intakeDownDegrees() {
        return new Trigger(()->false);
    }

    public Trigger finalShoot() {
        return m_controller.R1();
    }

    public Trigger maxSpeedShooter() {
        return new Trigger(()->false);
    }


    public Trigger amp(){
        return m_controller.L1();
    }

    public Trigger sourceIntake(){
        return  m_controller.square();
    }

    public Trigger hasNoteOverride(){
        return m_controller.share();
    }


    public Trigger toggleBeamBreakOne() {
        return new Trigger(()->false);
    }

    public Trigger toggleBeamBreakTwo() {
        return new Trigger(()->false);
    }

    public Trigger beambreakGamePiece() {
        return new Trigger(()->false);
    }

    public Trigger zeroAmp() {
        return m_controller.povLeft();
    }

    public Trigger preRev(){
        return m_controller.L2();
    }

    public void setRumble(double value, RumbleType type){
        m_controller.getHID().setRumble(type,value);
    }

}
