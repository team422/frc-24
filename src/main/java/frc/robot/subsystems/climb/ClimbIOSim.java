package frc.robot.subsystems.climb;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.Vision.ClimbConstants;

public class ClimbIOSim implements ClimbIO {
    DCMotorSim m_sim;

    public ClimbIOSim() {
        m_sim = new DCMotorSim(DCMotor.getFalcon500Foc(2), ClimbConstants.kClimbGearRatio, 0.5);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        m_sim.update(0.02);
        inputs.height = m_sim.getAngularPositionRad();
        inputs.velocity = m_sim.getAngularVelocityRadPerSec();
        inputs.currentAmps = m_sim.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double voltage) {
        m_sim.setInputVoltage(voltage);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        // TODO Auto-generated method stub
    }

}
