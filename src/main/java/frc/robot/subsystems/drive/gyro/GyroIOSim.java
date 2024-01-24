package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;

public class GyroIOSim implements GyroIO {
  // simulated FRC gyro
  AnalogGyroSim m_sim;

  public GyroIOSim() {

    m_sim = new AnalogGyroSim(1);
  }

  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.angle = m_sim.getAngle();

  }

  @Override
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(m_sim.getAngle());
  }

  @Override
  public void addAngle(Rotation2d angle) {
    m_sim.setAngle(m_sim.getAngle() + angle.getDegrees());
  }

  @Override
  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(0);
  }

}
