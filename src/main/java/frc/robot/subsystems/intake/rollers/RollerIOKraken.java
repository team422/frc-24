package frc.robot.subsystems.intake.rollers;

import com.ctre.phoenix6.hardware.TalonFX;

public class RollerIOKraken implements RollerIO {
    private TalonFX m_motor;

    public RollerIOKraken(int port) {
        m_motor = new TalonFX(port);
    }

    @Override
    public void updateInputs(RollerInputs inputs) {
        inputs.curVelocity = m_motor.getVelocity().getValueAsDouble();
        inputs.voltage = m_motor.getMotorVoltage().getValueAsDouble();
        inputs.outputCurrent = m_motor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }
}
