package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.drive.SwerveModuleIO.SwerveModuleInputs;

public interface SwerveModuleIO extends LoggedIO<SwerveModuleInputs> {
  @AutoLog
  public static class SwerveModuleInputs {
    public double turnAngleRads;
    public double turnRadsPerSecond;
    public double driveDistanceMeters;
    public double driveVelocityMetersPerSecond;
    public double voltageOutDrive;
    public double currentAmpsDrive;
    public double currentAmpsPerVelocity;
    public double driveVelocityMetersPerSecondAbs;
  }

  public SwerveModulePosition getPosition();

  public default void setUpModuleFirmware() {
  };

  public void resetDistance();

  public void syncTurningEncoder();

  public void resetEncoders();

  public Rotation2d getAngle();

  public void setDesiredState(SwerveModuleState swerveModuleState);

  public SwerveModuleState getState();

  public SwerveModuleState getAbsoluteState();

  public void setVoltage(double voltageDrive, double voltageTurn);

  default public void setVoltageDriveOnly(double voltageDrive, SwerveModulePosition voltageTurn) {

  };

  default public double getPowerUsage() {
    return 0.0;
  }

  public double getVoltage();

  public double getDriveCurrent();

  public void updateWheelSpeedCalculusSolver();

  public void updateCurrentCalculusSolver();

  public double getDeltaDriveCurrent();

  public double getDeltaWheelSpeed();

  public double getWheelSpeed();

  public void setBrakeMode(boolean mode);

  public void setVoltageDriveIgnoreTurn(double driveVoltage);

  public void setVoltageTurnIgnoreDrive(double turnVoltage);

}
