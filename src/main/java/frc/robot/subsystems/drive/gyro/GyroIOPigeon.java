package frc.robot.subsystems.drive.gyro;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Constants.DriveConstants;
// import frc.robot.subsystems.drive.PhoenixOdometryThread;
// import frc.robot.subsystems.drive.PhoenixOdometryThread;
import frc.robot.subsystems.drive.SparkMaxOdometryThread;

public class GyroIOPigeon implements GyroIO {
  Pigeon2 m_gyro;
  Pigeon2Configurator m_gyroConfig;
  private final double[] yprDegrees = new double[3];
  private final double[] xyzDps = new double[3];
  // private final Queue<Double> yawPositionQueue;
  private final StatusSignal<Double> yaw;
  private final StatusSignal<Double> yawVelocity;

  public GyroIOPigeon(int gyroPort, Rotation2d pitchAngle, boolean phoenixDrive) {
    m_gyro = new Pigeon2(gyroPort,"Drivetrain");
    m_gyroConfig = m_gyro.getConfigurator();
    yaw = m_gyro.getYaw();
    yaw.setUpdateFrequency(DriveConstants.kOdometryFrequency);
    yawVelocity = m_gyro.getAngularVelocityZWorld();

    m_gyro.getConfigurator().apply(new Pigeon2Configuration());
    m_gyro.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(DriveConstants.kOdometryFrequency);
    yawVelocity.setUpdateFrequency(250.0);
    // Pigeon2Configurator m_gyroConfig = m_gyro.getConfigurator();

    // Pigeon2Configuration gyroConfig = new Pigeon2Configuration();
    // gyroConfig.MountPose = Pigeon2.MountingYawPitchRoll;    
    // m_gyro.configMountPosePitch(pitchAngle.getDegrees());
    // m_gyro.configMountPose(0, 30, 0);

if (phoenixDrive) {
      // yawPositionQueue =
          // PhoenixOdometryThread.getInstance().registerSignal(m_gyro, m_gyro.getYaw());
    } else {
      // yawPositionQueue =
      //     SparkMaxOdometryThread.getInstance()
      //         .registerSignal(() -> m_gyro.getYaw().getValueAsDouble());
    }
  }

  @Override
  public Rotation2d getAngle() {
    return m_gyro.getRotation2d();
  }

  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
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

    inputs.rollVelocityRadPerSec = Rotation2d.fromDegrees(m_gyro.getAngularVelocityXDevice().getValue()).getRadians();
    inputs.pitchVelocityRadPerSec = Rotation2d.fromDegrees(m_gyro.getAngularVelocityYDevice().getValue()).getRadians();
    inputs.yawVelocityRadPerSec = Rotation2d.fromDegrees(m_gyro.getAngularVelocityZDevice().getValue()).getRadians();

    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());

    // inputs.odometryYawPositions =
    //     yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
    // yawPositionQueue.clear();

  }

  @Override
  public void addAngle(Rotation2d angle) {
    // m_gyro.add(angle.getDegrees());
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
