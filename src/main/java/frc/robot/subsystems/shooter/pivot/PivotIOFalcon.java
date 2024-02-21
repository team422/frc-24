package frc.robot.subsystems.shooter.pivot;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.ShooterConstants.PivotConstants;

public class PivotIOFalcon implements PivotIO {
    // Hardware
    private final TalonFX leaderTalon;
    private final TalonFX followerTalon;
    private final DutyCycleEncoder absoluteEncoder;

    // Status Signals
    private final StatusSignal<Double> armInternalPositionRotations;
    private final StatusSignal<Double> armVelocityRps;
    private final List<StatusSignal<Double>> armAppliedVoltage;
    private final List<StatusSignal<Double>> armOutputCurrent;
    private final List<StatusSignal<Double>> armTorqueCurrent;
    private final List<StatusSignal<Double>> armTempCelsius;

    // Control
    private final Slot0Configs controllerConfig;
    private final VoltageOut voltageControl =
        new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
    private final PositionTorqueCurrentFOC positionControl =
        new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

    

    private Rotation2d m_desiredAngle;



    public PivotIOFalcon(int primaryMotor, int secondaryMotor, int absoluteEncoderPort) {
        absoluteEncoder = new DutyCycleEncoder(absoluteEncoderPort);
        leaderTalon = new TalonFX(primaryMotor, "rio");
        followerTalon = new TalonFX(secondaryMotor, "rio");
        followerTalon.setControl(new Follower(primaryMotor, true));
        
        // Absolute encoder configs
        absoluteEncoder.setPositionOffset(absoluteEncoderPort);
        m_desiredAngle = getCurrentAngle();
        
        
        // Leader motor configs
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leaderConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leaderConfig.Feedback.SensorToMechanismRatio = 1.0;
        leaderConfig.Feedback.RotorToSensorRatio = PivotConstants.gearboxRatio;
        
        
        leaderTalon.setPosition(getCurrentAngle().getRotations(), 0.02);
        
        // Set up controller
        controllerConfig = new Slot0Configs().withKP(PivotConstants.kPivotP.get()).withKI(PivotConstants.kPivotI.get()).withKD(PivotConstants.kPivotD.get());
        leaderConfig.Slot0 = controllerConfig;

        leaderTalon.getConfigurator().apply(leaderConfig);


        // Follower configs
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        followerTalon.getConfigurator().apply(followerConfig);
        // Status signals
        armInternalPositionRotations = leaderTalon.getPosition();
        armVelocityRps = leaderTalon.getVelocity();
        armAppliedVoltage = List.of(leaderTalon.getMotorVoltage(), followerTalon.getMotorVoltage());
        armOutputCurrent = List.of(leaderTalon.getSupplyCurrent(), followerTalon.getSupplyCurrent());
        armTorqueCurrent = List.of(leaderTalon.getTorqueCurrent(), followerTalon.getTorqueCurrent());
        armTempCelsius = List.of(leaderTalon.getDeviceTemp(), followerTalon.getDeviceTemp());

        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            armInternalPositionRotations,
            armVelocityRps,
            armAppliedVoltage.get(0),
            armAppliedVoltage.get(1),
            armOutputCurrent.get(0),
            armOutputCurrent.get(1),
            armTorqueCurrent.get(0),
            armTorqueCurrent.get(1),
            armTempCelsius.get(0),
            armTempCelsius.get(1));

        // Optimize bus utilization
        leaderTalon.optimizeBusUtilization(1.0);
        followerTalon.optimizeBusUtilization(1.0);

        

    }

    @Override
    public void runSetpoint(Rotation2d angle, double feedforward) {
        m_desiredAngle = angle;
        leaderTalon.setControl(positionControl.withPosition(m_desiredAngle.getRotations()).withFeedForward(feedforward));
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.firstMotorConnected =
            BaseStatusSignal.refreshAll(
                    armInternalPositionRotations,
                    armVelocityRps,
                    armAppliedVoltage.get(0),
                    armOutputCurrent.get(0),
                    armTorqueCurrent.get(0),
                    armTempCelsius.get(0))
                .isOK();
        inputs.secondMotorConnected =
            BaseStatusSignal.refreshAll(
                    armAppliedVoltage.get(1),
                    armOutputCurrent.get(1),
                    armTorqueCurrent.get(1),
                    armTempCelsius.get(1))
                .isOK();

        inputs.armPositionRads = Units.rotationsToRadians(armInternalPositionRotations.getValue());
        inputs.armVelocityRadsPerSec = Units.rotationsToRadians(armVelocityRps.getValue());
        inputs.armAppliedVolts =
            armAppliedVoltage.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
        inputs.armCurrentAmps =
            armOutputCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
        inputs.armTorqueCurrentAmps =
            armTorqueCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
        inputs.armTempCelcius =
            armTempCelsius.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
        inputs.curAngle = getCurrentAngle().getDegrees();
        
    }


    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition());
    }
    
}