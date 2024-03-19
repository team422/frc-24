package frc.robot.subsystems.shooter.pivot;

import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.utils.LoggedTunableNumber;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants.ShooterPivotConstants;
import frc.robot.RobotState.RobotCurrentAction;

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
    private Slot0Configs controllerConfig;
    private final VoltageOut voltageControl =
        new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
    private final PositionTorqueCurrentFOC positionControl =
        new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
    private final PositionDutyCycle positionDutyCycle = 
        new PositionDutyCycle(0.0).withUpdateFreqHz(0.0);
        private final PositionVoltage positionVoltageCycle = 
        new PositionVoltage(0.0).withUpdateFreqHz(0.0).withEnableFOC(true);
    

    private Rotation2d m_desiredAngle;

    private ArmFeedforward armFF = new ArmFeedforward(0, 0, 0);

    private DCMotorSim m_pivotSim = new DCMotorSim(DCMotor.getFalcon500Foc(2),ShooterPivotConstants.gearboxRatio, 0.05);
    // FOR SIM ONLY
    private CANcoder m_canCoder;


    public PivotIOFalcon(int primaryMotor, int secondaryMotor, int absoluteEncoderPort) {
        absoluteEncoder = new DutyCycleEncoder(absoluteEncoderPort);
        leaderTalon = new TalonFX(primaryMotor, "rio");
        followerTalon = new TalonFX(secondaryMotor, "rio");
        
        // Absolute encoder configs
        absoluteEncoder.setPositionOffset(ShooterPivotConstants.kOffset);
        absoluteEncoder.setDistancePerRotation(-1);
        
        
        
        // Leader motor configs
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leaderConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leaderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leaderConfig.Feedback.SensorToMechanismRatio = ShooterPivotConstants.gearboxRatio;
        leaderConfig.Feedback.RotorToSensorRatio = 1.0;
        // enable foc
        
        
        
        
        // Set up controller
        controllerConfig = new Slot0Configs();
        controllerConfig.kP = ShooterPivotConstants.kPivotP.get();
        controllerConfig.kI = ShooterPivotConstants.kPivotI.get();
        controllerConfig.kD = ShooterPivotConstants.kPivotD.get();
        leaderConfig.Slot0 = controllerConfig;
        
        leaderTalon.setPosition(Rotation2d.fromRotations(absoluteEncoder.get()).getRotations(), 0.0);
        leaderTalon.getConfigurator().apply(leaderConfig);
        if(Robot.isSimulation()){
            m_canCoder = new CANcoder(60);
            m_canCoder.setPosition(0);
        }
        m_desiredAngle = getCurrentAngle();
        
        // Follower configs
        
        // leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        
        followerTalon.getConfigurator().apply(leaderConfig);
        followerTalon.setControl(new Follower(primaryMotor, false));
        // leaderTalon.setPosition(absoluteEncoder.get());
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

        // System.out.println(angle);
        feedforward = armFF.calculate(m_desiredAngle.getRadians(),getCurrentVelocity());
        leaderTalon.setControl(positionVoltageCycle.withPosition(m_desiredAngle.getRotations()).withFeedForward(feedforward).withUpdateFreqHz(0));
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        if(frc.robot.RobotState.getInstance().curAction.equals(RobotCurrentAction.kAmpLineup)){
            LoggedTunableNumber.ifChanged(hashCode(),()->{
                controllerConfig.kP = ShooterPivotConstants.kPivotAmpP.get();
                controllerConfig.kI = ShooterPivotConstants.kPivotAmpI.get();
                controllerConfig.kD = ShooterPivotConstants.kPivotAmpD.get();
                controllerConfig.kG = ShooterPivotConstants.kPivotkG.get();
                controllerConfig.kS = ShooterPivotConstants.kPivotkS.get();
                leaderTalon.getConfigurator().apply(controllerConfig);
                followerTalon.getConfigurator().apply(controllerConfig);
                armFF = new ArmFeedforward(ShooterPivotConstants.kPivotkS.get(), ShooterPivotConstants.kPivotkG.get(), ShooterPivotConstants.kPivotkV.get());
            }, ShooterPivotConstants.kPivotAmpP,ShooterPivotConstants.kPivotAmpI,ShooterPivotConstants.kPivotAmpD,ShooterPivotConstants.kUsingAmp,ShooterPivotConstants.kPivotkG,ShooterPivotConstants.kPivotkS,ShooterPivotConstants.kPivotkG,ShooterPivotConstants.kPivotkV,ShooterPivotConstants.kPivotkA);   
        }else{
            LoggedTunableNumber.ifChanged(hashCode(),()->{
                controllerConfig.kP = ShooterPivotConstants.kPivotP.get();
                controllerConfig.kI = ShooterPivotConstants.kPivotI.get();
                controllerConfig.kD = ShooterPivotConstants.kPivotD.get();
                controllerConfig.kG = ShooterPivotConstants.kPivotkG.get();
                controllerConfig.kS = ShooterPivotConstants.kPivotkS.get();
                armFF = new ArmFeedforward(ShooterPivotConstants.kPivotkS.get(), ShooterPivotConstants.kPivotkG.get(), ShooterPivotConstants.kPivotkV.get());
                leaderTalon.getConfigurator().apply(controllerConfig);
                followerTalon.getConfigurator().apply(controllerConfig);

            }, ShooterPivotConstants.kPivotP,ShooterPivotConstants.kPivotI,ShooterPivotConstants.kPivotD,ShooterPivotConstants.kUsingAmp,ShooterPivotConstants.kPivotkG,ShooterPivotConstants.kPivotkS,ShooterPivotConstants.kPivotkG,ShooterPivotConstants.kPivotkV,ShooterPivotConstants.kPivotkA);   
        }
        leaderTalon.setPosition(getCurrentAngle().getRotations(), 0.0);
        
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

        inputs.armPositionRads = Units.rotationsToDegrees(armInternalPositionRotations.getValue());
        inputs.absolutePositionDegrees = getCurrentAngle().getDegrees();
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
        inputs.desiredAngle = m_desiredAngle.getDegrees();
        if(Robot.isSimulation()){
            simulationPeriodic();
        }
        
    }


    public void simulationPeriodic(){
        TalonFXSimState lSimState = leaderTalon.getSimState();
        // System.out.println(lSimState.getMotorVoltage());
        m_pivotSim.setInputVoltage(lSimState.getMotorVoltage());

        m_pivotSim.update(0.02);

        leaderTalon.getSimState().addRotorPosition(m_pivotSim.getAngularVelocityRPM()*0.02/60.0);
        followerTalon.getSimState().addRotorPosition(m_pivotSim.getAngularVelocityRPM()*0.02/60.0);

        m_canCoder.getSimState().setRawPosition(m_pivotSim.getAngularPositionRotations()/ShooterPivotConstants.gearboxRatio);


    }


  
    
    @Override
    public Rotation2d getDesiredAngle() {
        return m_desiredAngle;
    }

    @Override
    public Rotation2d getCurrentAngle() {
        if(Robot.isSimulation()){
            return Rotation2d.fromRotations(m_canCoder.getAbsolutePosition().getValueAsDouble());
        }

        return Rotation2d.fromDegrees(Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition()-absoluteEncoder.getPositionOffset()+0.5).getDegrees() % 180);
    }

    @Override
    public double getCurrentVelocity() {
        return Rotation2d.fromRotations(armVelocityRps.getValueAsDouble()).getRadians();
    }
}