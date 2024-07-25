package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.drive.SwerveModuleIO.SwerveModuleInputs;

public interface SwerveModuleIO {
  @AutoLog
  public static class SwerveModuleInputs {
    public double turnAngleRads;
    public double turnRadsPerSecond;
    public double[] odometryDrivePositionsMeters = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    public double driveDistanceMeters;
    public double driveVelocityMetersPerSecond;
    public double voltageOutDrive;
    public double currentAmpsDrive;
    public double currentAmpsPerVelocity;
    public double driveVelocityMetersPerSecondAbs;
    public double angleDegrees;
    public double angleDegreesCanCoder;
    public double angleDegreesCanCoderPosition;
    public double currentAmpsSupplied;
    public double voltageSupplyDrive;
    public double voltageOutTurn;
    public boolean driveMotorConnected;
    public boolean turnMotorConnected;
    public double desiredAngleDegrees;
  }

  public default void updateInputs(SwerveModuleInputs inputs){};

  // public SwerveModulePosition getPosition();

  public default void setUpModuleFirmware() {
  };

  public default void resetDistance() {
  };

  public default void syncTurningEncoder() {
  };

  public default void resetEncoders() {};

  // public Rotation2d getAngle();

  public default void setDesiredState(SwerveModuleState swerveModuleState){};

  // public SwerveModuleState getState();

  public default void runDriveVelocitySetpoint(double velocityMetersPerSecond, double feedforwardVolts){};

  // public SwerveModuleState getAbsoluteState();

  public default void setVoltage(double voltageDrive, double voltageTurn){};

  default  public void setVoltageDriveOnly(double voltageDrive, SwerveModulePosition voltageTurn) {};

    
  default void runTurnPositionSetpoint(double angleRads) {};

  

  default public void setDesiredStateWithAcceleration(SwerveModuleState swerveModuleState, double acceleration){
    setDesiredState(swerveModuleState);
  }

  default public double getPowerUsage() {
    return 0.0;
  }

  default public void setCurrentLimit(double limit){
    return;
  }

  public default double getVoltage(){
    return 0;
  }

  public default double getDriveCurrent(){
    return 0;
  }

  public default void updateWheelSpeedCalculusSolver(){
    return;
  }

  public default void updateCurrentCalculusSolver(){
    return;
  }

  public default double getDeltaDriveCurrent(){
    return 0;
  }

  public default double getDeltaWheelSpeed(){
    return 0;
  }


  public default double getWheelSpeed(){
    return 0;
  }

  public default void setBrakeMode(boolean mode){
    return;
  }

  public default void setVoltageDriveIgnoreTurn(double driveVoltage){
    return;
  }

  public default void setVoltageTurnIgnoreDrive(double turnVoltage){
    return;
  }

  public default void setDriveVelocitySetpoint(double velocityRadsPerSec, double ffVolts) {
  };

  public default void setTurnPositionSetpoint(double angleRads) {
  };

  public default void setDrivePID(double p, double i, double d){
    return;
  }

  public default void setTurnPID(double p, double i, double d){
    return;
  }
  

  public default void setTurnFF(double ks, double kv, double ka){
    return;
  }

  public default void setDriveFF(double ks, double kv, double ka){
    return;
  }

  public default void lowerCurrentLimits(){
    return;
  }


  public default void runCharacterization(double turnSetpoint, double driveVoltage){
    return;
  }

  public default SwerveModuleState getSetpointState(){
    return new SwerveModuleState(0, new Rotation2d());
  }

  public default double getWheelRotations(){
    return 0.0;
  }

}
