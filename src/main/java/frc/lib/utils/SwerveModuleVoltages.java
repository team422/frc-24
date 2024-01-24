package frc.lib.utils;

public class SwerveModuleVoltages {
  public double m_driveVoltage;
  public double m_turnVoltage;
  public boolean m_driveOnly;
  public boolean m_turnOnly;

  public SwerveModuleVoltages(double driveVoltage, double turnVoltage, boolean driveOnly, boolean turnOnly) {
    m_driveVoltage = driveVoltage;
    m_turnVoltage = turnVoltage;
  }

  public SwerveModuleVoltages setDriveOnly(double driveVoltage) {
    return new SwerveModuleVoltages(driveVoltage, 0, true, false);
  }

  public SwerveModuleVoltages setTurnOnly(double turnVoltage) {
    return new SwerveModuleVoltages(0, turnVoltage, false, true);
  }

  public SwerveModuleVoltages setBothVoltages(double driveVoltage, double turnVoltage) {
    return new SwerveModuleVoltages(driveVoltage, turnVoltage, false, false);
  }

}
