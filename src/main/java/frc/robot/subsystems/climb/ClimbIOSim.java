package frc.robot.subsystems.climb;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.Vision.ClimbConstants;

public class ClimbIOSim implements ClimbIO {
    DCMotorSim m_sim;
    double m_voltage;

    public ClimbIOSim() {
        m_sim = new DCMotorSim(DCMotor.getFalcon500Foc(2), ClimbConstants.kClimbGearRatio, 0.5);
        m_voltage = 0;
    }

    @Override
    public void updateInputs(ClimbInputs inputs) {
        m_sim.update(0.02);
        inputs.height = m_sim.getAngularPositionRad();
        inputs.velocity = m_sim.getAngularVelocityRadPerSec();
        inputs.currentAmps = m_sim.getCurrentDrawAmps();
        inputs.outputVoltage = m_voltage;
    }

    @Override
    public void setVoltage(double voltage) {
        m_sim.setInputVoltage(voltage);
        m_voltage = voltage;
    }

    @Override
    public void setBrakeMode(boolean enabled) {
    }

}
