package frc.robot.subsystems.drive;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.compound.Diff_TorqueCurrentFOC_Velocity;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.lib.utils.TunableNumber;
import frc.robot.Constants;

public class SwerveModuleIOMK4Talon implements SwerveModuleIO {

  private TalonFX m_driveMotor;
  private TalonFX m_turnMotor;

  private CANcoder m_absoluteEncoder;

  private CANcoderConfiguration m_absoluteEncoderConfig;
  private CANcoderConfigurator m_absoluteEncoderConfigurator;

  private PIDController m_turnController;

  private FeedbackConfigs m_driveMotorConfig;
  private CurrentLimitsConfigs m_driveMotorCurrentLimitsConfigs;
  private FeedbackConfigs m_turnMotorConfig;
  private CurrentLimitsConfigs m_turnMotorCurrentLimitsConfigs;

  private Slot0Configs m_driveSlot0Configs;
  private Slot0Configs m_turnSlot0Configs;

  private double m_oldDriveCurrent;


  public static class ModuleConstants {
    public static final double kDriveConversionFactor = 1 / 22.0409;
    public static final double kTurnPositionConversionFactor = 21.428;
    public static final TunableNumber kDriveP = Constants.ModuleConstants.kDriveP;
    public static final TunableNumber kDriveI = Constants.ModuleConstants.kDriveI;
    public static final TunableNumber kDriveD = Constants.ModuleConstants.kDriveD;
    public static final TunableNumber kTurningP = Constants.ModuleConstants.kTurningP;
    public static final TunableNumber kTurningI = Constants.ModuleConstants.kTurningI;
    public static final TunableNumber kTurningD = Constants.ModuleConstants.kTurningD;
    // public static final TunableNumber kDriveFF = RobotContainer.robotConstants.kDriveFF;
  }

  public SwerveModuleIOMK4Talon(Integer driveMotorID, Integer turningMotorID, Integer absoluteEncoderID) {
    m_driveMotor = new TalonFX(driveMotorID);
    m_turnMotor = new TalonFX(turningMotorID);
    m_absoluteEncoder = new CANcoder(absoluteEncoderID);

    m_absoluteEncoderConfig = new CANcoderConfiguration();
    m_absoluteEncoderConfigurator = m_absoluteEncoder.getConfigurator();
    m_absoluteEncoderConfigurator.refresh(m_absoluteEncoderConfig);

    //  now we set gear ratios and positioning
    m_absoluteEncoderConfig.MagnetSensor.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
    m_absoluteEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    m_absoluteEncoderConfigurator.apply(m_absoluteEncoderConfig);
    m_driveMotorConfig = new FeedbackConfigs(); 
    m_driveMotorConfig.FeedbackRemoteSensorID = driveMotorID;
    m_driveMotorConfig.SensorToMechanismRatio = 5.35714286;
    m_driveMotor.getConfigurator().apply(m_driveMotorConfig);
    m_turnMotorConfig = new FeedbackConfigs(); 
    m_turnMotorConfig.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    m_turnMotorConfig.FeedbackRemoteSensorID = absoluteEncoderID;
    m_turnMotorConfig.SensorToMechanismRatio = (1);
    // m_turnMotorConfig.RotorToSensorRatio = 150/7;
    m_turnMotorConfig.RotorToSensorRatio = 7/150;
    m_turnController = new PIDController(ModuleConstants.kTurningP.get(), ModuleConstants.kTurningI.get(), ModuleConstants.kTurningD.get());
    m_turnController.enableContinuousInput(0, 1);
    
    
    m_driveMotor.getConfigurator().apply(m_turnMotorConfig);
    m_driveMotorCurrentLimitsConfigs = new CurrentLimitsConfigs();
    m_driveMotorCurrentLimitsConfigs.SupplyTimeThreshold = 0.5;
    m_driveMotorCurrentLimitsConfigs.SupplyCurrentThreshold = 75;
    m_driveMotorCurrentLimitsConfigs.SupplyCurrentLimit = 70;
    m_driveMotorCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
    m_driveMotor.getConfigurator().apply(m_driveMotorCurrentLimitsConfigs);
    m_turnMotorCurrentLimitsConfigs = new CurrentLimitsConfigs();
    m_turnMotorCurrentLimitsConfigs.SupplyTimeThreshold = 0.5;
    m_turnMotorCurrentLimitsConfigs.SupplyCurrentThreshold = 65;
    m_turnMotorCurrentLimitsConfigs.SupplyCurrentLimit = 60;
    m_turnMotorCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
    m_turnMotor.getConfigurator().apply(m_turnMotorCurrentLimitsConfigs);
    m_driveSlot0Configs = new Slot0Configs();
    m_driveSlot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    m_driveSlot0Configs.kP = ModuleConstants.kDriveP.get();
    m_driveSlot0Configs.kI = ModuleConstants.kDriveI.get();
    m_driveSlot0Configs.kD = ModuleConstants.kDriveD.get();
    m_driveMotor.getConfigurator().apply(m_driveSlot0Configs);
    m_turnSlot0Configs = new Slot0Configs();
    m_turnSlot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    m_turnSlot0Configs.kP = ModuleConstants.kTurningP.get();
    m_turnSlot0Configs.kI = ModuleConstants.kTurningI.get();
    m_turnSlot0Configs.kD = ModuleConstants.kTurningD.get();
    m_turnMotor.getConfigurator().apply(m_turnSlot0Configs);

    



  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    inputs.driveDistanceMeters = getDriveDistanceMeters();
    inputs.driveVelocityMetersPerSecond = getDriveVelocityMetersPerSecond();
    inputs.driveVelocityMetersPerSecondAbs = Math.abs(getDriveVelocityMetersPerSecond());
    inputs.turnAngleRads = getAngle().getRadians();
    inputs.turnRadsPerSecond = getTurnVelocity().getRadians();
    inputs.currentAmpsDrive = m_driveMotor.getTorqueCurrent().getValueAsDouble();
    inputs.currentAmpsSupplied = m_driveMotor.getSupplyCurrent().getValueAsDouble();
    inputs.voltageOutDrive = m_driveMotor.getMotorVoltage().getValueAsDouble();
    inputs.voltageOutTurn = m_turnMotor.getMotorVoltage().getValueAsDouble();
    inputs.voltageSupplyDrive = m_driveMotor.getSupplyVoltage().getValueAsDouble();
    inputs.currentAmpsPerVelocity = Math.abs(m_driveMotor.getTorqueCurrent().getValueAsDouble() / getDriveVelocityMetersPerSecond());
    inputs.angleDegreesCanCoderPosition = m_absoluteEncoder.getAbsolutePosition().getValue();

  }

  public Rotation2d getTurnVelocity(){
    return Rotation2d.fromDegrees(m_turnMotor.getVelocity().getValueAsDouble());
  }

  public double getDriveVelocityMetersPerSecond(){
    return m_driveMotor.getVelocity().getValueAsDouble();
  }

  public double getDriveDistanceMeters(){
    return m_driveMotor.getPosition().getValueAsDouble();
  }

  @Override
  public SwerveModulePosition getPosition() {
    
    return new SwerveModulePosition(getDriveDistanceMeters(),getTurnPosition());
  }

  public Rotation2d getTurnPosition(){
    return Rotation2d.fromDegrees(m_turnMotor.getPosition().getValueAsDouble());
  }

  @Override
  public void resetDistance() {
    m_driveMotor.setPosition(0);

  }

  @Override
  public void syncTurningEncoder() {
    //  in theory ABS enc -> turn enc by default

  }

  @Override
  public void resetEncoders() {
    m_absoluteEncoderConfig.MagnetSensor.MagnetOffset = m_absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    m_absoluteEncoderConfigurator.apply(m_absoluteEncoderConfig);
  }

  @Override
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(m_absoluteEncoder.getAbsolutePosition().getValueAsDouble());
  }

  @Override
  public void setDesiredState(SwerveModuleState swerveModuleState) {
    m_driveMotor.setControl(new VelocityTorqueCurrentFOC(swerveModuleState.speedMetersPerSecond).withSlot(0));
    double val = m_turnController.calculate(getAngle().getRotations(),Math.abs(swerveModuleState.angle.getRotations()));
    if(ModuleConstants.kTurningP.hasChanged()){
        m_turnController.setP(ModuleConstants.kTurningP.get());
    }
    System.out.println(m_turnController.calculate(getAngle().getRotations(),swerveModuleState.angle.getRotations()));
    System.out.println(val);
    System.out.println(getAngle().getRotations());
    System.out.println(swerveModuleState.angle.getRotations());
    m_turnMotor.setControl(new VoltageOut(val));
    
    // m_turnMotor.setControl(new VoltageOut(3));
  }

  @Override
  public void setDesiredStateWithAcceleration(SwerveModuleState swerveModuleState, double acceleration){
    m_driveMotor.setControl(new VelocityTorqueCurrentFOC(swerveModuleState.speedMetersPerSecond).withAcceleration(acceleration).withSlot(0));
    m_turnMotor.setControl(new PositionVoltage(swerveModuleState.angle.getDegrees()).withSlot(0));
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getAngle());
  }

  @Override
  public SwerveModuleState getAbsoluteState() {
    return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getAngle());
  }

  @Override
  public void setVoltage(double voltageDrive, double voltageTurn) {
    m_driveMotor.setControl(new VoltageOut(voltageDrive));
    m_turnMotor.setControl(new VoltageOut(voltageTurn)); 

  }

  @Override
  public double getVoltage() {
    
    return m_driveMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public double getDriveCurrent() {
    return m_driveMotor.getTorqueCurrent().getValueAsDouble();
  }

  @Override
  public void updateWheelSpeedCalculusSolver() {
    // TODO Auto-generated method stub
  }

  @Override
  public void updateCurrentCalculusSolver() {
    // TODO Auto-generated method stub

  }

  @Override
  public double getDeltaDriveCurrent() {
    double val = m_driveMotor.getTorqueCurrent().getValueAsDouble()-m_oldDriveCurrent;
    m_oldDriveCurrent = m_driveMotor.getTorqueCurrent().getValueAsDouble();
    return val;
  }

  @Override
  public double getDeltaWheelSpeed() {
    return m_driveMotor.getAcceleration().getValueAsDouble();
  }

  @Override
  public double getWheelSpeed() {
    return m_driveMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void setBrakeMode(boolean mode) {
    

  }

  @Override
  public void setVoltageDriveIgnoreTurn(double driveVoltage) {
    setVoltage(driveVoltage, 0);

  }

  @Override
  public void setVoltageTurnIgnoreDrive(double turnVoltage) {
    setVoltage(0,turnVoltage);

  }

}
