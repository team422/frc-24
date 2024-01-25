package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private double kMinHeight;
    private double kMaxHeight;
    private ClimbIO m_io;
    private ClimbIOInputsAutoLogged m_inputs;
    public double m_desiredHeight;
    private ProfiledPIDController m_controller;

    public Climb(ClimbIO io, ProfiledPIDController controller, double minHeight, double maxHeight) {
        m_io = io;
        m_inputs = new ClimbIOInputsAutoLogged();

        m_controller = controller;
        m_controller.setTolerance(0.05);
        m_controller.setIntegratorRange(-0.5, 0.5);

        kMinHeight = minHeight;
        kMaxHeight = maxHeight;
    }

    @Override
    public void periodic() {
        double pidVoltage = m_controller.calculate(m_inputs.height, m_desiredHeight);

        m_io.updateInputs(m_inputs);
        Logger.processInputs("Climber", m_inputs);

        m_io.setVoltage(pidVoltage);
    }

    public void setDesiredHeight(double height) {
        m_desiredHeight = MathUtil.clamp(height, kMinHeight, kMaxHeight);
    }

    public double getHeight() {
        return m_inputs.height;
    }

    public Command setDesiredHeightCommand(double height) {
        return Commands.runOnce(() -> setDesiredHeight(height));
    }

    public Command moveCommand(double delta) {
        return run(() -> setDesiredHeight(m_inputs.height + delta));
    }

}
