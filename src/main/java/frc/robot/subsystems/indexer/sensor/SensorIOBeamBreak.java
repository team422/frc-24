package frc.robot.subsystems.indexer.sensor;

import edu.wpi.first.wpilibj.DigitalInput;

public class SensorIOBeamBreak implements SensorIO {
    private DigitalInput m_beamBreak;

    public SensorIOBeamBreak(int port) {
        m_beamBreak = new DigitalInput(port);
    }

    public void updateInputs(SensorInputs inputs) {
        inputs.broken = m_beamBreak.get();
    }

    public boolean getBroken() {
        return m_beamBreak.get();
    }

}
