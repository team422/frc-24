package frc.robot.subsystems.indexer.wheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class WheelIOSim implements WheelIO {
    private DCMotorSim m_motorSim;
    private double m_voltage;
    private final double kWheelRadius = 0.05;

    public WheelIOSim() {
        DCMotor gearbox = DCMotor.getNEO(1);
        double gearing = 3;
        double jKgMetersSquared = 0.0001;
        m_motorSim = new DCMotorSim(gearbox, gearing, jKgMetersSquared);
        m_voltage = 0;
    }

    @Override
    public void updateInputs(WheelInputs inputs) {
        m_motorSim.update(0.02);
        inputs.voltage = getVoltage();
        inputs.current = getCurrent();
        inputs.angularVelocity = getAngularVelocity();
        inputs.linearVelocity = getLinearVelocity();
        inputs.position = getPosition();
    }

    @Override
    public void setVoltage(double voltage) {
        m_voltage = voltage;
        m_motorSim.setInputVoltage(voltage);
    }

    @Override
    public double getVoltage() {
        return m_voltage;
    }

    @Override
    public double getCurrent() {
        return m_motorSim.getCurrentDrawAmps();
    }

    @Override
    public double getAngularVelocity() {
        return m_motorSim.getAngularVelocityRadPerSec();
    }

    @Override
    public double getLinearVelocity() {
        return m_motorSim.getAngularVelocityRadPerSec() * kWheelRadius;
    }

    @Override
    public double getPosition() {
        return m_motorSim.getAngularPositionRad();
    }
}
