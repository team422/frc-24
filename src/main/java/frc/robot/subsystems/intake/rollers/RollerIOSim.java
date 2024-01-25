package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollerIOSim implements RollerIO {
    private DCMotorSim m_sim;
    private double m_voltage;

    public RollerIOSim() {
        DCMotor gearbox = DCMotor.getVex775Pro(2);
        double gearing = 3;
        double jKgMetersSquared = 1;
        m_sim = new DCMotorSim(gearbox, gearing, jKgMetersSquared);
        m_voltage = 0;
    }

    @Override
    public void updateInputs(RollerInputs inputs) {
        m_sim.update(0.02);
        inputs.curVelocity = m_sim.getAngularVelocityRadPerSec();
        inputs.voltage = m_voltage;
        inputs.outputCurrent = m_sim.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double voltage) {
        m_sim.setInputVoltage(voltage);
        m_voltage = voltage;
    }


}
