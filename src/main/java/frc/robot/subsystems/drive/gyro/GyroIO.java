package frc.robot.subsystems.drive.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.drive.gyro.GyroIO.GyroInputs;

public interface GyroIO {
  @AutoLog
  public static class GyroInputs {
    public double angle = 0.0;
    public double pitch = 0.0;
    public boolean connected = false;
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public double rollPositionRad = 0.0;
    public double pitchPositionRad = 0.0;
    public double yawPositionRad = 0.0;
    public double rollVelocityRadPerSec = 0.0;
    public double xMetersSecondSquare = 0.0;
    public double yMetersSecondSquare = 0.0;
    public double zMetersSecondSquare = 0.0;
    public double pitchVelocityRadPerSec = 0.0;
    public double yawVelocityRadPerSec = 0.0;
    public Rotation2d yawPosition = new Rotation2d();
  }

  // public Rotation2d getAngle();

  // public void addAngle(Rotation2d angle);

  // public Rotation2d getPitch();

  public default double getAccel() {
    return 0;
  }

  public default double getAccelX() {
    return 0;
  };

  public default double getAccelY() {
    return 0;
  };
  default void updateInputs(GyroInputs inputs) {}

}
