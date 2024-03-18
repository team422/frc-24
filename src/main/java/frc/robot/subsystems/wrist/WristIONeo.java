package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class WristIONeo implements WristIO {
    private CANSparkMax m_motor;

    public WristIONeo(int port) {
        m_motor = new CANSparkMax(port, MotorType.kBrushless);
    }

    @Override
    public void setVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }
}
