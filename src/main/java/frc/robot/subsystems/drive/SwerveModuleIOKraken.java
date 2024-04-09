package frc.robot.subsystems.drive;

import java.util.Queue;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.utils.LoggedTunableNumber;
import frc.lib.utils.TunableNumber;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Robot;
public class SwerveModuleIOKraken implements SwerveModuleIO {
    private TalonFX m_driveMotor;
    private TalonFX m_turnMotor;

    private CANcoder m_turnEncoder;

    // Status Signals
  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveSupplyCurrent;
  private final StatusSignal<Double> driveTorqueCurrent;

  private final StatusSignal<Double> turnPosition;
  private final Supplier<Rotation2d> turnAbsolutePosition;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnSupplyCurrent;
  private final StatusSignal<Double> turnTorqueCurrent;

    
    TalonFXConfiguration driveConfig;
    TalonFXConfiguration turnConfig;

    private final Slot0Configs driveFeedbackConfig = new Slot0Configs();
  private final Slot0Configs turnFeedbackConfig = new Slot0Configs();
    PIDController mController;

    // private final Queue<Double> drivePositionQueue;
    // private final Queue<Double> turnPositionQueue;

      // Control
  private final VoltageOut driveVoltage = new VoltageOut(0).withUpdateFreqHz(0);
  private final VoltageOut turnVoltage = new VoltageOut(0).withUpdateFreqHz(0);
  private final TorqueCurrentFOC driveCurrent = new TorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final TorqueCurrentFOC turnCurrent = new TorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final VelocityVoltage driveVelocityControl =
      new VelocityVoltage(0).withEnableFOC(true);
  private final PositionTorqueCurrentFOC turnPositionControl =
      new PositionTorqueCurrentFOC(0);

    private final PIDController mDriveController;

    private PIDController mTurnController;
    private ArmFeedforward mTurnControllerFF;

    private final VelocityVoltage driveVelocityVoltage = new VelocityVoltage(0.0);

    private double mDesSpeed = 0;
    private double mDesAngle = 0;

    private final PositionVoltage turnPositionControlVoltage = new PositionVoltage(0);
    private final PositionVoltage positionControl =
      new PositionVoltage(0).withEnableFOC(true);
  private final NeutralOut driveNeutral = new NeutralOut().withUpdateFreqHz(0);
  private final NeutralOut turnNeutral = new NeutralOut().withUpdateFreqHz(0);

    private  DCMotorSim mDriveSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), ModuleConstants.kDriveConversionFactor, 0.4);
    private  DCMotorSim mTurnSim = new DCMotorSim(DCMotor.getFalcon500Foc(1), ModuleConstants.kTurnPositionConversionFactor, 0.025);
    double angleRadsSet = 0;


    public SwerveModuleIOKraken(TalonFX drivePort, TalonFX turnPort, CANcoder cancoderId, boolean turnMotorInverted) {
        // m_driveMotor = new TalonFX(drivePort, "Drivetrain");
        // m_turnMotor = new TalonFX(turnPort, "Drivetrain");
        // m_turnEncoder = new CANcoder(cancoderId,"Drivetrain");

        m_driveMotor = drivePort;
        m_turnMotor = turnPort;
        m_turnEncoder = cancoderId;

        driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.SupplyCurrentLimit = 65.0;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = 150;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.Voltage.PeakForwardVoltage = 12.0;
        driveConfig.Voltage.PeakReverseVoltage = 12.0;
        
        turnConfig = new TalonFXConfiguration();
        turnConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
        turnConfig.CurrentLimits.StatorCurrentLimit = 150.0;
        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        // statorCurrentLimit = 150.0;
        
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        // turnConfig.Voltage.PeakForwardVoltage = 12.0;
        // turnConfig.Voltage.PeakReverseVoltage = -12.0;
        turnConfig.MotorOutput.Inverted = turnMotorInverted
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
        
        driveConfig.Feedback.SensorToMechanismRatio = ModuleConstants.kDriveConversionFactor;
        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // driveConfig.Feedback.RotorToSensorRatio = ModuleConstants.kDriveConversionFactor;
        // setDrivePID(.2, 0,0);

        mController = new PIDController(2/(2*Math.PI),0,0);

        CANcoderConfiguration canCfg = new CANcoderConfiguration();
        m_turnEncoder.getConfigurator().refresh(canCfg);
        Logger.recordOutput("Swerve Module 1",canCfg.MagnetSensor.MagnetOffset);
        canCfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        canCfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        // canCfg.MagnetSensor.MagnetOffset = 0.0;
        // setTurnPID(.3, 0, 0);
        if(Robot.isSimulation()){
            // turnConfig.Feedback.SensorToMechanismRatio = 1;
            // turnConfig.Feedback.RotorToSensorRatio = 150/7;
        }else {
            // turnConfig.Feedback.SensorToMechanismRatio = 1;
            m_turnEncoder.getConfigurator().apply(canCfg);
            turnConfig.Feedback.SensorToMechanismRatio = 1;
            turnConfig.Feedback.RotorToSensorRatio = 7/150 ;
        }
        turnConfig.Feedback.FeedbackRemoteSensorID = m_turnEncoder.getDeviceID();
        turnConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        // mDriveController = new PIDController(m, turnPort, cancoderId)
        
        // for (int i = 0; i < 4; i++) {
        //     boolean error = m_driveMotor.getConfigurator().apply(driveConfig, 0.1) == StatusCode.OK;
        //     setDriveBrakeMode(true);
        //     error = error && (m_turnMotor.getConfigurator().apply(turnConfig, 0.1) == StatusCode.OK);
        //     setTurnBrakeMode(true);
            
        //     if (!error) break;
        // }
        // setDrivePID(ModuleConstants.kDriveP.get(), ModuleConstants.kDriveI.get(), ModuleConstants.kDriveD.get());
        mDriveController = new PIDController(0.3,0,0);
        mTurnController = new PIDController(0.3, 0.0, 0.1);
        mTurnController.enableContinuousInput(-Math.PI, Math.PI);


        drivePosition = m_driveMotor.getRotorPosition();
        turnPosition = m_turnMotor.getPosition();
        BaseStatusSignal.setUpdateFrequencyForAll(250.0, drivePosition, turnPosition);

        driveVelocity = m_driveMotor.getVelocity();
        driveAppliedVolts = m_driveMotor.getMotorVoltage();
        driveSupplyCurrent = m_driveMotor.getSupplyCurrent();
        driveTorqueCurrent = m_driveMotor.getTorqueCurrent();
        
        turnAbsolutePosition = ()->{return Rotation2d.fromRotations(m_turnEncoder.getPosition().getValueAsDouble());};
        turnVelocity = m_turnMotor.getVelocity();
        turnAppliedVolts = m_turnMotor.getMotorVoltage();
        turnSupplyCurrent = m_turnMotor.getSupplyCurrent();
        turnTorqueCurrent = m_turnMotor.getTorqueCurrent();
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            driveVelocity,
            driveAppliedVolts,
            driveSupplyCurrent,
            driveTorqueCurrent,
            turnVelocity,
            turnAppliedVolts,
            turnSupplyCurrent,
            turnTorqueCurrent
        );

// STUFF NEEDED FOR OUR ODO
        // drivePositionQueue =
        // PhoenixOdometryThread.getInstance().registerSignal(m_driveMotor, drivePosition);
        // turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(m_turnMotor, turnPosition);
        // Reset turn position to absolute encoder position
        // m_turnMotor.setPosition(turnAbsolutePosition.get().getRotations(), 1.0);
        // drivePositionQueue = null;
        // turnPositionQueue = null;
    
        // BaseStatusSignal.setUpdateFrequencyForAll(50.0, drivePosition, turnPosition);

        m_driveMotor.optimizeBusUtilization(1.0);
    m_turnMotor.optimizeBusUtilization(1.0);
    if(Robot.isSimulation()){
        m_turnEncoder.setPosition(0.0);
    }
        
    }

    public void lowerCurrentLimits(){
        CurrentLimitsConfigs driveLimits = new CurrentLimitsConfigs();
        m_driveMotor.getConfigurator().refresh(driveLimits);
        driveLimits.SupplyCurrentLimit = 45;
        m_driveMotor.getConfigurator().apply(driveLimits);
    }


    
    public void simulationPeriodic(){

        TalonFXSimState simDrive =  m_driveMotor.getSimState();
        TalonFXSimState simTurn = m_turnMotor.getSimState();
        // System.out.println(simTurn.getMotorVoltage());
        Logger.recordOutput("Motor sim", simTurn.getMotorVoltage());
        mDriveSim.setInputVoltage(Math.min(12,Math.max(simDrive.getMotorVoltage(),-12)));
        mTurnSim.setInputVoltage(Math.min(12,Math.max(simTurn.getMotorVoltage(),-12)));


        mDriveSim.update(0.02);
        mTurnSim.update(0.02);

        // dont use with command drive train
        // m_driveMotor.getSimState().setRawRotorPosition(mDriveSim.getAngularPositionRotations() * ModuleConstants.kDriveConversionFactor);
        // m_turnMotor.getSimState().setRawRotorPosition(mTurnSim.getAngularPositionRotations());
        // m_turnEncoder.getSimState().setRawPosition(mTurnSim.getAngularPositionRotations());
        // m_driveMotor.getSimState().setRotorVelocity(mDriveSim.getAngularVelocityRPM() * ModuleConstants.kDriveConversionFactor/60.0);
        // m_turnMotor.getSimState().setRotorVelocity(mTurnSim.getAngularVelocityRPM()/60.0);




    }


    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        // BaseStatusSignal.refreshAll(
        //         drivePosition,
        //         driveVelocity,
        //         driveAppliedVolts,
        //         driveSupplyCurrent,
        //         driveTorqueCurrent,
        //         turnPosition,
        //         turnVelocity,
        //         turnAppliedVolts,
        //         turnSupplyCurrent,
        //         turnTorqueCurrent);

        LoggedTunableNumber.ifChanged(hashCode(), ()->{
            setDrivePID(ModuleConstants.kDriveP.get(), ModuleConstants.kDriveI.get(), ModuleConstants.kDriveD.get());
        },ModuleConstants.kDriveP,ModuleConstants.kDriveI, ModuleConstants.kDriveD);
        

        

        

                inputs.driveMotorConnected =
        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveAppliedVolts,
                driveSupplyCurrent,
                driveTorqueCurrent)
            .isOK();
    inputs.turnMotorConnected =
        BaseStatusSignal.refreshAll(
                turnPosition, turnVelocity, turnAppliedVolts, turnSupplyCurrent, turnTorqueCurrent)
            .isOK();

        inputs.driveDistanceMeters = getDriveDistanceMeters();
        inputs.driveVelocityMetersPerSecond = getDriveVelocityMetersPerSecond();
        inputs.turnAngleRads = Units.radiansToDegrees(turnPosition.getValueAsDouble());
        inputs.turnRadsPerSecond = getTurnVelocityRadsPerSecond();
        inputs.angleDegrees = getAngle().getDegrees();
        inputs.desiredAngleDegrees = Rotation2d.fromRadians(angleRadsSet).getDegrees();
        inputs.currentAmpsDrive = getDriveCurrent();
        inputs.angleDegreesCanCoder = turnAbsolutePosition.get().getDegrees();
        inputs.currentAmpsSupplied = driveSupplyCurrent.getValue();
        inputs.voltageSupplyDrive = driveAppliedVolts.getValue();
        inputs.driveVelocityMetersPerSecondAbs = Math.abs(inputs.driveVelocityMetersPerSecond);
        inputs.voltageOutTurn = turnAppliedVolts.getValue();



    // inputs.odometryDrivePositionsMeters =
    // drivePositionQueue.stream()
    //     .mapToDouble(
    //         signalValue -> (Units.rotationsToRadians(signalValue)/2) * ModuleConstants.kWheelDiameterMeters / ModuleConstants.kDriveGearRatio)
    //         .toArray();
            // signalValue -> signalValue)
// inputs.odometryTurnPositions =
//     turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
//     drivePositionQueue.clear();
//     turnPositionQueue.clear();

    if(RobotBase.isSimulation()){
        simulationPeriodic();
    }

    }

    public void runCharacterization(double turnSetpoint, double driveVoltage){
        m_driveMotor.setControl(new VoltageOut(driveVoltage).withEnableFOC(true));
        runTurnPositionSetpoint(turnSetpoint);
    }
    
    public double getDriveVelocityMetersPerSecond() {
        return driveVelocity.getValueAsDouble();
    }
    public double getDriveDistanceMeters() {
        return (Units.rotationsToRadians(drivePosition.getValue())/2) * ModuleConstants.kWheelDiameterMeters / ModuleConstants.kDriveGearRatio;
    }

    public double convertDriveMetersToRadians(double meters) {
        return (meters / ModuleConstants.kWheelDiameterMeters) * (2 * Math.PI);
    }

    public double getTurnVelocityRadsPerSecond() {
        return turnVelocity.getValue() * Math.PI * 2;
    }


    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistanceMeters(), getAngle());
    }


    @Override
    public void resetDistance() {
        m_driveMotor.setPosition(0);
    }


    @Override
    public void syncTurningEncoder() {
        // do nothing, they are fused
    }


    @Override
    public void resetEncoders() {
        // set the offset to the current position
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = m_turnEncoder.getAbsolutePosition().getValue();
        m_turnEncoder.getConfigurator().apply(config,0.1);
    }


    @Override
    public Rotation2d getAngle() {
        // return Rotation2d.fromRadians(turnPosition.getValueAsDouble());
        return Rotation2d.fromRadians(MathUtil.angleModulus(turnAbsolutePosition.get().getRadians()));
    }


    @Override
    public void setDesiredState(SwerveModuleState swerveModuleState) {
        // System.out.println(swerveModuleState);
        // System.out.println(convertDriveMetersToRadians(swerveModuleState.speedMetersPerSecond));
        setDriveVelocitySetpoint(swerveModuleState.speedMetersPerSecond, 0);
        
        setTurnPositionSetpoint(swerveModuleState.angle.getRadians());
    }


    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getAngle());
    }


    @Override
    public SwerveModuleState getAbsoluteState() {
        // TODO Auto-generated method stub
        return new SwerveModuleState(getDriveVelocityMetersPerSecond(),getAngle());
        // throw new UnsupportedOperationException("Unimplemented method 'getAbsoluteState'");
    }


    @Override
    public void setCurrentLimit(double limit) {
        driveConfig.CurrentLimits.SupplyCurrentLimit = limit;
        m_driveMotor.getConfigurator().apply(driveConfig.CurrentLimits,0.1);
    }

    @Override
    public void setVoltage(double voltageDrive, double voltageTurn) {
        m_driveMotor.setControl(new VoltageOut(voltageDrive));
        m_turnMotor.setControl(new VoltageOut(voltageTurn));
    }


    @Override
    public double getVoltage() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVoltage'");
    }


    @Override
    public double getDriveCurrent() {
        return driveSupplyCurrent.getValue();
    }


    @Override
    public void updateWheelSpeedCalculusSolver() {
        // no time :(
    }


    @Override
    public void updateCurrentCalculusSolver() {
        // no time :(
    }


    @Override
    public double getDeltaDriveCurrent() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getDeltaDriveCurrent'");
    }


    @Override
    public double getDeltaWheelSpeed() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getDeltaWheelSpeed'");
    }


    @Override
    public double getWheelSpeed() {
        return getDriveVelocityMetersPerSecond();
    }


    @Override
    public void setBrakeMode(boolean mode) {
        m_driveMotor.setNeutralMode(mode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        m_turnMotor.setNeutralMode(mode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }


    @Override
    public void setVoltageDriveIgnoreTurn(double driveVoltage) {
        m_driveMotor.setControl(new VoltageOut(driveVoltage).withEnableFOC(true));
    }


    @Override
    public void setVoltageTurnIgnoreDrive(double turnVoltage) {
        m_turnMotor.setControl(new VoltageOut(turnVoltage).withEnableFOC(true));
    }

    @Override
    public void setDriveVelocitySetpoint(double velocityRadsPerSec, double ffVolts) {
        mDesSpeed = velocityRadsPerSec;
        double velocityRotationsPerSec = Units.radiansToRotations(velocityRadsPerSec);

        if (DriveConstants.useTorqueCurrentFOC) {

            // m_driveMotor.setControl(new VelocityTorqueCurrentFOC(velocityRotationsPerSec).withSlot(0));
            System.out.println("drive rads" + getDriveVelocityMetersPerSecond() + "wanted rads" + velocityRadsPerSec);
            m_driveMotor.setControl(driveVelocityControl.withVelocity(velocityRotationsPerSec));
        } else {
            // m_driveMotor.setControl(new VoltageOut(5));

            // double m_val = mController.calculate(getDriveVelocityMetersPerSecond(),velocityRadsPerSec);
            // m_driveMotor.setControl(new VoltageOut(velocityRadsPerSec*4));
            m_driveMotor.setControl(
                    new VelocityVoltage(velocityRotationsPerSec)
                    .withEnableFOC(true).withUpdateFreqHz(0).withSlot(0));
                            // .withFeedForward(ffVolts)
        }
    }

    @Override
    public void setTurnPositionSetpoint(double angleRads) {
        double angleRotations = Units.radiansToRotations(angleRads);
        mDesAngle = angleRads;
        if (DriveConstants.useTorqueCurrentFOC) {
            if (DriveConstants.useMotionMagic) {
                m_turnMotor.setControl(new MotionMagicTorqueCurrentFOC(angleRotations));
            } else {
                m_turnMotor.setControl(new PositionTorqueCurrentFOC(angleRotations));
            }
        } else {
            if (DriveConstants.useMotionMagic) {
                m_turnMotor.setControl(new MotionMagicVoltage(angleRotations).withEnableFOC(true));
            } else {
                m_turnMotor.setControl(new PositionVoltage(angleRotations).withEnableFOC(true));
            }
        }
    }

    @Override
  public void runDriveVelocitySetpoint(double velocityMetersPerSec, double feedForward) {
    // System.out.println(velocityMetersPerSec*2);
    // System.out.println(driveVelocity.getValueAsDouble());
    // m_driveMotor.setControl(
    //     driveVelocityVoltage
    //         .withVelocity(velocityRadsPerSec).withSlot(0));

    double pidVal = mDriveController.calculate(driveVelocity.getValueAsDouble(), velocityMetersPerSec);
    if (velocityMetersPerSec < 1){
        pidVal/=5;   
    }
    m_driveMotor.setControl(new VoltageOut(pidVal + feedForward).withEnableFOC(true));
    double rotations = metersPerSecondToRotationsPerSecond(velocityMetersPerSec);
    // m_driveMotor.setControl(driveVelocityControl.withVelocity(rotations).withFeedForward(feedForward + pidVal));

  }

  public double metersPerSecondToRotationsPerSecond(double velocityMetersPerSecond){
    return Units.radiansToRotations(velocityMetersPerSecond/ModuleConstants.kWheelDiameterMeters);

  }
    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        driveFeedbackConfig.kP = kP;
        driveFeedbackConfig.kI = kI;
        driveFeedbackConfig.kD = kD;
     
        // m_driveMotor.getConfigurator().apply(driveFeedbackConfig,0.1);

        mDriveController.setPID(kP, kI, kD);
    }

    @Override
  public void runTurnPositionSetpoint(double angleRads) {
    // System.out.println("running turn position setpoint" + angleRads);
    angleRadsSet = angleRads;
    if (Robot.isReal()){
        // angleRads = Units.degreesToRadians(ModuleConstants.currentAngleToGoTo.get());
        double turnFF = ModuleConstants.turnStatic.get();
        if(getAngle().getRadians()  - MathUtil.angleModulus(angleRads)  < 0){
            turnFF *=-1;
        }

        // Logger.recordOutput("ff des angle",getAngle().getRadians());
        // Logger.recordOutput("ff actual angle",MathUtil.angleModulus(angleRads));
        // Logger.recordOutput("ff val",turnFF);


        

        m_turnMotor.setControl(positionControl.withPosition(Units.radiansToRotations(angleRads)).withEnableFOC(true).withFeedForward(turnFF));
        // m_turnMotor.setControl(new VoltageOut(-mTurnController.calculate(getAngle().getRadians(x ), angleRads)).withEnableFOC(true));
        // m_turnMotor.setControl(new VoltageOut(3).withEnableFOC(true));
    } else {
        // m_turnMotor.setControl(positionControl.withPosition(Units.radiansToRotations(angleRads)));
        m_turnMotor.setControl(new VoltageOut(-mTurnController.calculate(turnAbsolutePosition.get().getRadians() % (Math.PI *2), angleRads)).withEnableFOC(true));
    }

  }
    @Override
    public void setTurnPID(double kP, double kI, double kD) {
        // turnConfig.Slot0.kP = kP;
        // turnConfig.Slot0.kI = kI;
        // turnConfig.Slot0.kD = kD;
        
        // m_turnMotor.getConfigurator().apply(turnConfig,0.01);
        turnFeedbackConfig.kP = kP;
    turnFeedbackConfig.kI = kI;
    turnFeedbackConfig.kD = kD;
    // m_turnMotor.getConfigurator().apply(turnFeedbackConfig, 0.1);

    mTurnController.setPID(kP, kI, kD);
    // mTurnController.setIZone(3);
    }

    @Override
    public void setTurnFF(double kS, double kV, double kA) {
        turnFeedbackConfig.kS = kS;
        turnFeedbackConfig.kV = kV;
        turnFeedbackConfig.kA = kA;
        // m_turnMotor.getConfigurator().apply(turnConfig,0.1);
        // mTurnControllerFF = new ArmFeedforward(kS, kV, kA);
    }

    @Override
    public void setDriveFF(double kS,double kV, double kA){
        driveConfig.Slot0.kS = kS;
        driveConfig.Slot0.kV = kV;
        driveConfig.Slot0.kA = kA;
        // m_driveMotor.getConfigurator().apply(driveConfig,0.1);

    }

    public void setDriveBrakeMode(boolean mode) {
        m_driveMotor.setNeutralMode(mode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public void setTurnBrakeMode(boolean mode) {
        m_turnMotor.setNeutralMode(mode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public void stop() {
        m_driveMotor.setControl(new VoltageOut(0).withEnableFOC(true));
        m_turnMotor.setControl(new VoltageOut(0).withEnableFOC(true));
    }

    public SwerveModuleState getSetpointState(){
        return new SwerveModuleState(mDesSpeed,Rotation2d.fromRadians(mDesAngle));
    }

    public double getWheelRotations(){
        return m_driveMotor.getRotorPosition().getValueAsDouble()/ModuleConstants.kDriveGearRatio;
    }
    

}