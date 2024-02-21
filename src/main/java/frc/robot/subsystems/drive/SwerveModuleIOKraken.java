package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
public class SwerveModuleIOKraken implements SwerveModuleIO {
    private TalonFX m_driveMotor;
    private TalonFX m_turnMotor;

    private CANcoder m_turnEncoder;

    private StatusSignal<Double> drivePosition;
    private StatusSignal<Double> driveVelocity;
    private StatusSignal<Double> driveAppliedVolts;
    private StatusSignal<Double> driveSupplyCurrent;
    private StatusSignal<Double> driveTorqueCurrent;

    private StatusSignal<Double> turnPosition;
    private Supplier<Rotation2d> turnAbsolutePosition;
    private StatusSignal<Double> turnVelocity;
    private StatusSignal<Double> turnAppliedVolts;
    private StatusSignal<Double> turnSupplyCurrent;
    private StatusSignal<Double> turnTorqueCurrent;

    TalonFXConfiguration driveConfig;
    TalonFXConfiguration turnConfig;

    Slot0Configs driveFeedbackConfigs;
    Slot0Configs turnFeedbackConfigs;
    PIDController mController;

    public SwerveModuleIOKraken(int drivePort, int turnPort, int cancoderId, boolean turnMotorInverted) {
        m_driveMotor = new TalonFX(drivePort);
        m_turnMotor = new TalonFX(turnPort);
        m_turnEncoder = new CANcoder(cancoderId);

        driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.Voltage.PeakForwardVoltage = 12.0;
        driveConfig.Voltage.PeakReverseVoltage = 12.0;
        driveFeedbackConfigs = new Slot0Configs();
        turnFeedbackConfigs = new Slot0Configs();
        turnConfig = new TalonFXConfiguration();
        turnConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnConfig.Voltage.PeakForwardVoltage = 12.0;
        turnConfig.Voltage.PeakReverseVoltage = -12.0;
        turnConfig.MotorOutput.Inverted = turnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        
        driveConfig.Feedback.SensorToMechanismRatio = ModuleConstants.kDriveConversionFactor;
        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // driveConfig.Feedback.RotorToSensorRatio = ModuleConstants.kDriveConversionFactor;
        setDrivePID(.2, 0,0);
        mController = new PIDController(2/(2*Math.PI),0,0);
        turnConfig.Feedback.SensorToMechanismRatio = 1;
        turnConfig.Feedback.RotorToSensorRatio = 7/150;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;


        for (int i = 0; i < 4; i++) {
            boolean error = m_driveMotor.getConfigurator().apply(driveConfig, 0.1) == StatusCode.OK;
            setDriveBrakeMode(true);
            error = error && (m_turnMotor.getConfigurator().apply(turnConfig, 0.1) == StatusCode.OK);
            setTurnBrakeMode(true);
            
            if (!error) break;
        }

        driveVelocity = m_driveMotor.getVelocity();
        driveAppliedVolts = m_driveMotor.getMotorVoltage();
        driveSupplyCurrent = m_driveMotor.getSupplyCurrent();
        driveTorqueCurrent = m_driveMotor.getTorqueCurrent();
        turnAbsolutePosition = ()->{return Rotation2d.fromRotations(m_turnEncoder.getAbsolutePosition().getValueAsDouble());};
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

        drivePosition = m_driveMotor.getPosition();
        turnPosition = m_turnMotor.getPosition();
        BaseStatusSignal.setUpdateFrequencyForAll(250.0, drivePosition, turnPosition);
        
    }


    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveAppliedVolts,
                driveSupplyCurrent,
                driveTorqueCurrent,
                turnPosition,
                turnVelocity,
                turnAppliedVolts,
                turnSupplyCurrent,
                turnTorqueCurrent);

        inputs.driveDistanceMeters = getDriveDistanceMeters();
        inputs.driveVelocityMetersPerSecond = getDriveVelocityMetersPerSecond();
        inputs.turnAngleRads = getAngle().getRadians();
        inputs.turnRadsPerSecond = getTurnVelocityRadsPerSecond();
        inputs.angleDegrees = getAngle().getDegrees();
        inputs.currentAmpsDrive = getDriveCurrent();
        inputs.angleDegreesCanCoder = turnAbsolutePosition.get().getDegrees();
        inputs.currentAmpsSupplied = driveSupplyCurrent.getValue();
        inputs.voltageSupplyDrive = driveAppliedVolts.getValue();
        inputs.driveVelocityMetersPerSecondAbs = Math.abs(inputs.driveVelocityMetersPerSecond);
        inputs.voltageOutTurn = turnAppliedVolts.getValue();


    }
    
    public double getDriveVelocityMetersPerSecond() {
        return driveVelocity.getValue() * ModuleConstants.kDriveConversionFactor;
    }
    public double getDriveDistanceMeters() {
        return drivePosition.getValue()   ;
    }

    public double convertDriveMetersToRadians(double meters) {
        return (meters);
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
        m_turnEncoder.getConfigurator().apply(config,0.01);
    }


    @Override
    public Rotation2d getAngle() {
        return turnAbsolutePosition.get();
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
    public void setVoltage(double voltageDrive, double voltageTurn) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
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
        double velocityRotationsPerSec = Units.radiansToRotations(velocityRadsPerSec);
        if (DriveConstants.useTorqueCurrentFOC) {

            m_driveMotor.setControl(new VelocityTorqueCurrentFOC(velocityRotationsPerSec).withSlot(0));
        } else {
            // m_driveMotor.setControl(new VoltageOut(5));

            // double m_val = mController.calculate(getDriveVelocityMetersPerSecond(),velocityRadsPerSec);
            System.out.println("drive rads" + getDriveVelocityMetersPerSecond() + "wanted rads" + velocityRadsPerSec);
            m_driveMotor.setControl(new VoltageOut(velocityRadsPerSec*4));
            // m_driveMotor.setControl(
            //         new VelocityVoltage(velocityRotationsPerSec)
            //         .withEnableFOC(true));
                            // .withFeedForward(ffVolts)
        }
    }

    @Override
    public void setTurnPositionSetpoint(double angleRads) {
        double angleRotations = Units.radiansToRotations(angleRads);
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

    public void setDrivePID(double kP, double kI, double kD) {
        driveFeedbackConfigs.kP = kP;
        driveFeedbackConfigs.kI = kI;
        driveFeedbackConfigs.kD = kD;
        driveFeedbackConfigs.kS = 2;
        driveFeedbackConfigs.kV = 0;
        driveFeedbackConfigs.kA = 0;
     
        m_driveMotor.getConfigurator().apply(driveFeedbackConfigs,0.01);
    }

    public void setTurnPID(double kP, double kI, double kD) {
        turnConfig.Slot0.kP = kP;
        turnConfig.Slot0.kI = kI;
        turnConfig.Slot0.kD = kD;
        
        m_turnMotor.getConfigurator().apply(turnConfig,0.01);
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
    

}