package frc.robot.subsystems.led;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.hardwareprofiler.ProfiledSubsystem;

public class Led extends ProfiledSubsystem {
    private AddressableLED m_LEDStrip;
    private AddressableLEDBuffer m_LEDStripBuffer;
    private LedState m_state;
    private double m_gamepieceTimeout = -1;
    private int m_gamepieceFlashCount = 0;

    public static enum LedState {
        OFF,
        NO_GAME_PIECE,
        FLASHING,
        AUTO_DRIVING_TO_NOTE,
        GAME_PIECE,
        SHOOTER_READY,
        DISABLED,
        
    }

    public Led(int port, int length) {
        m_LEDStrip = new AddressableLED(port);
        m_LEDStripBuffer = new AddressableLEDBuffer(length);
        m_LEDStrip.setLength(length);
        m_LEDStrip.start();
        m_state = LedState.OFF;
    }

    @Override
    public void periodic() {
        double start = HALUtil.getFPGATime();

        m_LEDStrip.setData(m_LEDStripBuffer);

        if (m_state == LedState.FLASHING && m_gamepieceTimeout != -1 && m_gamepieceTimeout < Timer.getFPGATimestamp()) {
        // if (m_state == LedState.FLASHING) {
            // flash sequence
            if (m_LEDStripBuffer.getLED(0).equals(Color.kBlack)) {
                setSolidColor(Color.kGreen);
            } else {
                setSolidColor(Color.kBlack);
            }
            m_gamepieceTimeout = Timer.getFPGATimestamp() + 0.1;
        }

        Logger.recordOutput("LED/State", m_state.toString());
        Logger.recordOutput("LED/Color", m_LEDStripBuffer.getLED(0).toString());
        
        Logger.recordOutput("LoggedRobot/LEDPeriodic", (HALUtil.getFPGATime()-start)/1000);
    }

    public void setSolidColor(Color color) {
        double start = HALUtil.getFPGATime();
        for (int i = 0; i < m_LEDStripBuffer.getLength(); i++) {
            m_LEDStripBuffer.setLED(i, color);
        }
        m_LEDStrip.setData(m_LEDStripBuffer);
        Logger.recordOutput("LoggedRobot/LEDTime", (HALUtil.getFPGATime()-start)/1000);
    }

    public void setState(LedState state) {
        if (state == m_state) {
            return;
        }
        switch (state) {
            case OFF:
                setSolidColor(Color.kBlack);
                break;
            case NO_GAME_PIECE:
                setSolidColor(Color.kRed);
                break;
            case FLASHING:
                m_gamepieceFlashCount = 0;
                m_gamepieceTimeout = Timer.getFPGATimestamp() + 0.1;
                break;
            case GAME_PIECE:
                setSolidColor(Color.kGreen);
                break;
            case SHOOTER_READY:
                setSolidColor(Color.kBlue);
                break;
            case DISABLED:
                setSolidColor(Color.kMagenta);
                break;
            case AUTO_DRIVING_TO_NOTE:
                setSolidColor(Color.kYellow);
                break;
        }
        m_state = state;
    }

    public LedState getState() {
        return m_state;
    }
}
