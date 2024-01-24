package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.utils.CalculusSolver;
import frc.robot.Constants.ModuleConstants;

public class SwerveModuleIOSim implements SwerveModuleIO {
  // private SwerveModuleState m_curState;
  // private SwerveModuleState m_desState;
  // private SwerveModulePosition m_curPos;
  private DCMotorSim m_driveMotor;
  private PIDController m_driveController;
  private SimpleMotorFeedforward m_driveFeedforward;

  private DCMotorSim m_turnMotor;
  private PIDController m_turnController;

  private double m_voltageDrive;

  public CalculusSolver m_wheelSpeedCalculusSolver;
  public CalculusSolver m_currentCalculusSolver;

  public SwerveModuleIOSim() {
    // m_curState = new SwerveModuleState();
    // m_desState = new SwerveModuleState();
    // m_curPos = new SwerveModulePosition();
    m_driveMotor = new DCMotorSim(DCMotor.getNEO(1), ModuleConstants.kDriveGearRatio, ModuleConstants.kDriveJ);
    m_driveController = new PIDController(ModuleConstants.kDriveP.get(), ModuleConstants.kDriveI.get(),
        ModuleConstants.kDriveP.get());
    m_driveFeedforward = new SimpleMotorFeedforward(ModuleConstants.kDriveKS.get(), ModuleConstants.kDriveKV.get(),
        ModuleConstants.kDriveKA.get());

    m_turnMotor = new DCMotorSim(DCMotor.getNEO(1), ModuleConstants.kTurnPositionConversionFactor,
        ModuleConstants.kTurningJ);
    m_turnController = new PIDController(ModuleConstants.kTurningPSim.get(), ModuleConstants.kTurningISim.get(),
        ModuleConstants.kTurningDSim.get());

    m_wheelSpeedCalculusSolver = new CalculusSolver(50);
    m_currentCalculusSolver = new CalculusSolver(50);

  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveMotor.getAngularPositionRad() * ModuleConstants.kDriveConversionFactor,
        getAngle());
  }

  public void setVoltage(double voltageDrive, double voltageTurn) {
    m_voltageDrive = voltageDrive;
    m_driveMotor.setInputVoltage(voltageDrive);
    m_turnMotor.setInputVoltage(voltageTurn);
  }

  public void setVoltageDriveOnly(double voltageDrive, SwerveModulePosition position) {
    m_voltageDrive = voltageDrive;

    double currentAngle = getAngle().getRadians();

    double turnPID = m_turnController.calculate(currentAngle, position.angle.getRadians());
    setVoltage(voltageDrive, turnPID);
  }

  public void resetDistance() {
    m_driveMotor = new DCMotorSim(DCMotor.getNEO(1), ModuleConstants.kDriveGearRatio, ModuleConstants.kDriveJ);
  }

  public void syncTurningEncoder() {
  }

  public void resetEncoders() {

  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(m_turnMotor.getAngularPositionRad() % (2 * Math.PI));
  }

  public void setDesiredState(SwerveModuleState swerveModuleState) {
    double desiredSpeed = swerveModuleState.speedMetersPerSecond * ModuleConstants.kDriveConversionFactor;
    double desiredAngle = swerveModuleState.angle.getRadians();
    double currentSpeed = getSpeed();
    double currentAngle = getAngle().getRadians();
    double driveFF = m_driveFeedforward.calculate(desiredSpeed);
    double drivePID = m_driveController.calculate(currentSpeed, desiredSpeed);
    double turnPID = m_turnController.calculate(currentAngle, desiredAngle);
    // System.out
    //     .println("DriveFF: " + driveFF + " DrivePID: " + drivePID + " TurnPID: " + turnPID);
    setVoltage(driveFF, turnPID);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getSpeed(), getAngle());
  }

  public double getSpeed() {
    // System.out.println("Speed: " + m_driveMotor.getAngularVelocityRadPerSec() * ModuleConstants.kDriveConversionFactor);
    return m_driveMotor.getAngularVelocityRadPerSec() * ModuleConstants.kDriveConversionFactor;
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    // double oldAngleRads = m_curPos.angle.getRadians();
    // double updatedAngle = MathUtil.interpolate(m_curState.angle.getRadians(), m_desState.angle.getRadians(), 1);
    // m_curState.angle = Rotation2d.fromRadians(updatedAngle);
    // m_curState.speedMetersPerSecond = m_desState.speedMetersPerSecond;

    // m_curPos.distanceMeters += m_curState.speedMetersPerSecond * 0.02;
    // m_curPos.angle = m_curState.angle;
    // inputs.turnAngleRads = m_curPos.angle.getRadians();

    // inputs.driveDistanceMeters = m_curPos.distanceMeters;
    // inputs.driveVelocityMetersPerSecond = m_curState.speedMetersPerSecond;
    // inputs.turnRadsPerSecond = (m_curPos.angle.getRotations() - oldAngleRads) / 0.02;

    inputs.turnAngleRads = getAngle().getRadians();
    inputs.driveDistanceMeters = getPosition().distanceMeters;
    inputs.driveVelocityMetersPerSecond = getSpeed();
    inputs.turnRadsPerSecond = m_turnMotor.getAngularVelocityRadPerSec();
    inputs.currentAmpsDrive = m_driveMotor.getCurrentDrawAmps();
    inputs.voltageOutDrive = m_voltageDrive;
    m_driveMotor.update(0.02);
    m_turnMotor.update(0.02);

  }

  @Override
  public SwerveModuleState getAbsoluteState() {
    return getState();
  }

  @Override
  public double getPowerUsage() {
    return m_driveMotor.getCurrentDrawAmps()
        * BatterySim.calculateDefaultBatteryLoadedVoltage(m_driveMotor.getCurrentDrawAmps())
        + m_turnMotor.getCurrentDrawAmps()
            * BatterySim.calculateDefaultBatteryLoadedVoltage(m_turnMotor.getCurrentDrawAmps());
  }

  @Override
  public double getDriveCurrent() {
    return m_driveMotor.getCurrentDrawAmps();
  }

  @Override
  public double getVoltage() {
    return m_voltageDrive;
  }

  @Override
  public void updateCurrentCalculusSolver() {
    m_currentCalculusSolver.addPoint(getDriveCurrent(), getVoltage());
  }

  @Override
  public void updateWheelSpeedCalculusSolver() {
    m_wheelSpeedCalculusSolver.addPoint(getSpeed(), getVoltage());
  }

  @Override
  public double getDeltaDriveCurrent() {
    return m_currentCalculusSolver.getInstantaneousDerivative();
  }

  @Override
  public double getDeltaWheelSpeed() {
    return m_wheelSpeedCalculusSolver.getInstantaneousDerivative();
  }

  @Override
  public double getWheelSpeed() {
    return getSpeed();
  }

  @Override
  public void setBrakeMode(boolean mode) {
    return;
  }

  @Override
  public void setVoltageDriveIgnoreTurn(double driveVoltage) {
    m_driveMotor.setInputVoltage(driveVoltage);

  }

  @Override
  public void setVoltageTurnIgnoreDrive(double turnVoltage) {
    m_turnMotor.setInputVoltage(turnVoltage);

  }

}
