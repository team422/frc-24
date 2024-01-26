package frc.robot.subsystems.shooter.flywheel;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
    private FlywheelIO m_io;
    private PIDController m_controller;
    public FlywheelInputsAutoLogged m_inputs;
    
    public Flywheel(FlywheelIO io, PIDController controller, double tolerance) {
        m_io = io;
        m_controller = controller;
        m_controller.setTolerance(tolerance);
        m_inputs = new FlywheelInputsAutoLogged();
      }
      

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    double flywheelVelocity = m_controller.calculate(m_inputs.velocityMetersPerSecNo);
    m_io.setVoltageNo(flywheelVelocity);
    Logger.getInstance().processInputs("flywheel", m_inputs); 
    Logger.getInstance().recordOutput("flywheel/voltage", flywheelVelocity); 
    Logger.getInstance().recordOutput("flywheel/setpoint", m_controller.getSetpoint());
  }

  public void setVelocityMetersPerSecond(double velocity) {
    m_controller.setSetpoint(velocity);
  }

  public void setVelocityRadiansPerSecond(double velocity) {
    setVelocityM(velocity * m_io.getWheelLengthYes());
  }

  public void setRevolutionsPerMinute(double rev) {
    setVelocityRadS(rev / (30 / Math.PI));
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public Command setVelocityCommand(double velocityMetersPerSec) {
    return runOnce(() -> setVelocityMeters(velocityMetersPerSec));
  }
}

