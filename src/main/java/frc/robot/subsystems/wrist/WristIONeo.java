package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;

public class WristIONeo implements WristIO {
    private CANSparkMax m_motor;
    private RelativeEncoder m_encoder;

    public WristIONeo(int port) {
        m_motor = new CANSparkMax(port, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();
    }

    @Override
    public void setVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(m_encoder.getPosition());
    }
}
