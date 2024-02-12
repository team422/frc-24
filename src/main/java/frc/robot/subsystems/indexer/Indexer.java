package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.lib.hardwareprofiler.ProfiledSubsystem;
import frc.robot.subsystems.indexer.sensor.SensorIO;
import frc.robot.subsystems.indexer.sensor.SensorInputsAutoLogged;
import frc.robot.subsystems.indexer.wheel.WheelIO;
import frc.robot.subsystems.indexer.wheel.WheelInputsAutoLogged;

public class Indexer extends ProfiledSubsystem {
    private WheelIO m_wheelIO;
    private SensorIO m_sensorIOUp;
    private SensorIO m_sensorIODown;
    public WheelInputsAutoLogged m_wheelInputs;
    public SensorInputsAutoLogged m_sensorInputsUp;
    public SensorInputsAutoLogged m_sensorInputsDown;

    private ProfiledPIDController m_controller;

    public Indexer(WheelIO wheelIO, SensorIO sensorIOUp, SensorIO sensorIODown, ProfiledPIDController controller, double tolerance) {
        m_wheelIO = wheelIO;
        m_sensorIOUp = sensorIOUp;
        m_sensorIODown = sensorIODown;
        m_controller = controller;
        m_controller.setTolerance(tolerance);

        m_wheelInputs = new WheelInputsAutoLogged();
        m_sensorInputsUp = new SensorInputsAutoLogged();
        m_sensorInputsDown = new SensorInputsAutoLogged();
    }

    public void periodic() {
        m_wheelIO.updateInputs(m_wheelInputs);
        m_sensorIOUp.updateInputs(m_sensorInputsUp);
        m_sensorIODown.updateInputs(m_sensorInputsDown);

        double pidVoltage = m_controller.calculate(m_wheelInputs.position);

        m_wheelIO.setVoltage(pidVoltage);

        Logger.processInputs("Indexer/Wheel", m_wheelInputs);
        Logger.processInputs("Indexer/SensorUp", m_sensorInputsUp);
        Logger.processInputs("Indexer/SensorDown", m_sensorInputsDown);

        Logger.recordOutput("Indexer/Voltage", pidVoltage);
        Logger.recordOutput("Indexer/Desired", m_controller.getGoal().position);
    }

    public void setSetpoint(double setpoint) {
        m_controller.setGoal(setpoint);
    }

    public boolean withinTolerance() {
        return m_controller.atGoal();
    }


}
