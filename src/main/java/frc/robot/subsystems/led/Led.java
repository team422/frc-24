package frc.robot.subsystems.led;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.hardwareprofiler.ProfiledSubsystem;

public class Led extends ProfiledSubsystem {
    private AddressableLED m_LEDStrip;
    private AddressableLEDBuffer m_LEDStripBuffer;
    private LedState m_state;

    public static enum LedState {
        OFF,
        NO_GAME_PIECE,
        GAME_PIECE,
        SHOOTER_READY
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

    public void updateState(LedState state) {
        switch (state) {
            case OFF:
                setSolidColor(Color.kBlack);
                break;
            case NO_GAME_PIECE:
                setSolidColor(Color.kRed);
                break;
            case GAME_PIECE:
                setSolidColor(Color.kYellow);
                break;
            case SHOOTER_READY:
                setSolidColor(new Color(0x00 / 255.0, 0xb2 / 255.0, 0x59 / 255.0));
                // mech tech green
                // values divided by 255.0 to convert to 0-1 range, there was an issue with the values being ints last year
                break;
        }
    }
}
