package frc.robot.subsystems.indexer.wheel;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class WheelIONeo implements WheelIO {
    private CANSparkMax m_motor;
    private RelativeEncoder m_encoder;

    public WheelIONeo(int port) {
        m_motor = new CANSparkMax(port, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();
    }

    @Override
    public void updateInputs(WheelInputs inputs) {
        inputs.voltage = getVoltage();
        inputs.current = getCurrent();
        inputs.angularVelocity = getAngularVelocity();
        inputs.linearVelocity = getLinearVelocity();
        inputs.position = getPosition();
    }

    @Override
    public void setVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }

    @Override
    public double getVoltage() {
        return m_motor.getBusVoltage();
    }

    @Override
    public double getCurrent() {
        return m_motor.getOutputCurrent();
    }

    @Override
    public double getAngularVelocity() {
        return m_encoder.getVelocity();
    }

    @Override
    public double getLinearVelocity() {
        return m_encoder.getVelocity();
    }

    @Override
    public double getPosition() {
        return m_encoder.getPosition();
    }
}
