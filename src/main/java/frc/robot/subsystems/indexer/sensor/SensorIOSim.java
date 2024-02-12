package frc.robot.subsystems.indexer.sensor;

public class SensorIOSim implements SensorIO {
    private boolean m_broken;

    public SensorIOSim() {
        m_broken = true;
    }

    @Override
    public void updateInputs(SensorInputs inputs) {
        inputs.broken = getBroken();
    }

    @Override
    public boolean getBroken() {
        return m_broken;
    }

    @Override
    public void toggleBroken() {
        m_broken = !m_broken;
    }
}
