package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Wrist {
    private WristIO m_io;
    private final double kVoltage;

    public Wrist(WristIO io, double voltage) {
        m_io = io;
        kVoltage = voltage;
    }

    public void moveUp() {
        m_io.setVoltage(kVoltage);
    }

    public void moveDown() {
        m_io.setVoltage(-kVoltage);
    }

    public Command moveUpCommand() {
        return Commands.runOnce(this::moveUp);
    }

    public Command moveDownCommand() {
        return Commands.runOnce(() -> moveDown());
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> m_io.setVoltage(0.0));
    }
}
