package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    public WristIO m_io;
    private PIDController m_controller;
    private Rotation2d m_desiredAngle;

    public Wrist(WristIO io, PIDController controller) {
        m_io = io;
        m_controller = controller;
        m_desiredAngle = Rotation2d.fromDegrees(0);
    }

    @Override
    public void periodic() {
        double voltage = m_controller.calculate(m_io.getAngle().getRadians(), m_desiredAngle.getRadians());
        m_io.setVoltage(voltage);
        Logger.recordOutput("Wrist/Voltage", voltage);
        Logger.recordOutput("Wrist/Angle", m_io.getAngle().getDegrees());
    }

    public void setDesiredAngle(Rotation2d angle) {
        m_desiredAngle = angle;
    }

    public Command setDesiredAngleCommand(Rotation2d angle) {
        return Commands.runOnce(() -> setDesiredAngle(angle));
    }

    
}
