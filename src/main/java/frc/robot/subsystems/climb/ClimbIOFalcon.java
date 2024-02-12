package frc.robot.subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;

public class ClimbIOFalcon implements ClimbIO {
    private TalonFX m_leader;
    private TalonFX m_follower;
    private boolean m_brakeMode;

    public ClimbIOFalcon(int leaderPort, int followerPort) {
        m_leader = new TalonFX(leaderPort);
        m_follower = new TalonFX(followerPort);
        // m_follower.follow(m_leader);
    }

    @Override
    public void updateInputs(ClimbInputs inputs) {
        inputs.height = m_leader.getPosition().getValueAsDouble();
        inputs.velocity = m_leader.getVelocity().getValueAsDouble();
        inputs.outputVoltage = m_leader.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = m_leader.getSupplyCurrent().getValueAsDouble();
        inputs.brake = m_brakeMode;
    }

    @Override
    public void setVoltage(double voltage) {
        m_leader.setVoltage(voltage);
        m_follower.setVoltage(voltage);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        if (m_brakeMode == enabled) {
            return;
        }
        m_leader.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        m_follower.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        m_brakeMode = enabled;
    }

}
