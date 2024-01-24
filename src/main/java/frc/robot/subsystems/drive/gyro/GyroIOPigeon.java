package frc.robot.subsystems.drive.gyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class GyroIOPigeon implements GyroIO {
  Pigeon2 m_gyro;
  Pigeon2Configurator m_gyroConfig;
  private final double[] yprDegrees = new double[3];
  private final double[] xyzDps = new double[3];

  public GyroIOPigeon(int gyroPort, Rotation2d pitchAngle) {
    m_gyro = new Pigeon2(gyroPort);
    m_gyroConfig = m_gyro.getConfigurator();
    // Pigeon2Configurator m_gyroConfig = m_gyro.getConfigurator();

    // Pigeon2Configuration gyroConfig = new Pigeon2Configuration();
    // gyroConfig.MountPose = Pigeon2.MountingYawPitchRoll;    
    // m_gyro.configMountPosePitch(pitchAngle.getDegrees());
    // m_gyro.configMountPose(0, 30, 0);

  }

  @Override
  public Rotation2d getAngle() {
    return m_gyro.getRotation2d();
  }

  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.angle = getAngle().getDegrees();
    inputs.pitch = getPitch().getDegrees();
    // m_gyro.(yprDegrees);
    // m_gyro.getRawGyro(xyzDps);
    // inputs.connected = m_gyro.getFaultField();
    // inputs.rollPositionRad = Units.degreesToRadians(yprDegrees[1]);
    // inputs.pitchPositionRad = Units.degreesToRadians(-yprDegrees[2]);
    // inputs.yawPositionRad = Units.degreesToRadians(yprDegrees[0]);
    // inputs.rollVelocityRadPerSec = Units.degreesToRadians(xyzDps[1]);
    // inputs.pitchVelocityRadPerSec = Units.degreesToRadians(-xyzDps[0]);
    // inputs.yawVelocityRadPerSec = Units.degreesToRadians(xyzDps[2]);
    Rotation3d rotation = m_gyro.getRotation3d();
    inputs.rollPositionRad = rotation.getX();
    inputs.pitchPositionRad = rotation.getY();
    inputs.yawPositionRad = rotation.getZ();

    inputs.rollVelocityRadPerSec = m_gyro.getAngularVelocityXDevice().getValue();
    inputs.pitchVelocityRadPerSec = m_gyro.getAngularVelocityYDevice().getValue();
    inputs.yawVelocityRadPerSec = m_gyro.getAngularVelocityZDevice().getValue();



  }

  @Override
  public void addAngle(Rotation2d angle) {
    // m_gyro.addYaw(angle.getDegrees());
    m_gyroConfig.setYaw(angle.plus(m_gyro.getRotation2d()).getDegrees());

  }

  @Override
  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(m_gyro.getPitch().getValue());
  }

  public void reset() { // pls never run this
    m_gyro.reset();
  }

  // public double getAccelX() {
  //   return m_accel.getX();
  // }

  // public double getAccelY() {
  //   return m_accel.getY();
  // }

  // public double getAccel() {
  //   return m_accel.getA();
  // }

}
