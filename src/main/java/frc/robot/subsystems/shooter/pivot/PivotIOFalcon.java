package frc.robot.subsystems.shooter.pivot;

import java.util.ArrayList;
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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.utils.LoggedTunableNumber;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants.ShooterPivotConstants;
import frc.robot.RobotState.RobotCurrentAction;
import frc.robot.utils.CtreBaseRefreshManager;

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

    private int counter = 0;

    private PIDController mPositionController = new PIDController(0,0,0);

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

    // private DCMotorSim m_pivotSim = new DCMotorSim(DCMotor.getFalcon500Foc(2),ShooterPivotConstants.gearboxRatio, 55.5);
    // FOR SIM ONLY
    private SingleJointedArmSim m_armSim = new SingleJointedArmSim(DCMotor.getFalcon500Foc(2), ShooterPivotConstants.gearboxRatio,.02,.06,Rotation2d.fromDegrees(11).getRadians(),Rotation2d.fromDegrees(84).getRadians(),false,Rotation2d.fromDegrees(11).getRadians());
    private CANcoder m_canCoder;
    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();

    public PivotIOFalcon(int primaryMotor, int secondaryMotor, int absoluteEncoderPort) {
        absoluteEncoder = new DutyCycleEncoder(absoluteEncoderPort);
        leaderTalon = new TalonFX(primaryMotor, "rio");
        followerTalon = new TalonFX(secondaryMotor, "rio");
        
        // Absolute encoder configs
        absoluteEncoder.setPositionOffset(ShooterPivotConstants.kOffset);
        absoluteEncoder.setDistancePerRotation(-1);
        
        
        
        // Leader motor configs
        
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
            250,
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

        ArrayList<StatusSignal> signals = new ArrayList<>();
        signals.add(armInternalPositionRotations);
        signals.add(armVelocityRps);
        signals.add(armAppliedVoltage.get(0));
        signals.add(armAppliedVoltage.get(1));
        signals.add(armOutputCurrent.get(0));
        signals.add(armOutputCurrent.get(1));
        signals.add(armTorqueCurrent.get(0));
        signals.add(armTorqueCurrent.get(1));
        signals.add(armTempCelsius.get(0));
        signals.add(armTempCelsius.get(1));

        CtreBaseRefreshManager.getInstance().addSignals(signals);


        // Optimize bus utilization
        leaderTalon.optimizeBusUtilization(1.0);
        followerTalon.optimizeBusUtilization(1.0);
        leaderTalon.setPosition(getCurrentAngle().getRotations(), 0.002);

        

    }

    @Override
    public void runSetpoint(Rotation2d angle, double feedforward) {
        m_desiredAngle = angle;

        // System.out.println(angle);
        feedforward = armFF.calculate(m_desiredAngle.getRadians(),getCurrentAngle().getRadians());
        if (Robot.isSimulation()){
            feedforward = 0;
        }
        // leaderTalon.setControl(positionVoltageCycle.withPosition(m_desiredAngle.getRotations()).withFeedForward(feedforward).withUpdateFreqHz(0));
        leaderTalon.setControl(new VoltageOut(mPositionController.calculate(getCurrentAngle().getRadians(), m_desiredAngle.getRadians())+feedforward).withEnableFOC(true));
    }


    @Override
    public void clearI(){
        // mPositionController.setIntegratorRange(-0.07,0.07);
        // mPositionController.reset();
        mPositionController = new PIDController(ShooterPivotConstants.kPivotPDirect.get(), ShooterPivotConstants.kPivotIDirect.get(), ShooterPivotConstants.kPivotDDirect.get());

    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        LoggedTunableNumber.ifChanged(hashCode(),()->{
            mPositionController.setP(ShooterPivotConstants.kPivotPDirect.get());
            mPositionController.setI(ShooterPivotConstants.kPivotIDirect.get());
            mPositionController.setD(ShooterPivotConstants.kPivotDDirect.get());
        }, ShooterPivotConstants.kPivotPDirect,ShooterPivotConstants.kPivotIDirect,ShooterPivotConstants.kPivotDDirect);

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

        // RobotCurrentAction curAction = frc.robot.RobotState.getInstance().curAction;
        // if(curAction.equals(RobotCurrentAction.kRevAndAlign) || curAction.equals(RobotCurrentAction.kAutoShoot) || curAction.equals(RobotCurrentAction.kAutoSOTM) || curAction.equals(RobotCurrentAction.kAmpShoot) ){
        //     if(!Robot.isSimulation()){
        //     counter +=1;
        //     if(counter % 10 == 0){
        //         leaderTalon.setPosition(getCurrentAngle().getRotations(), 0.002);
        //         counter = 0;
        //     }
            
        // }
    // }
        // if(!Robot.isSimulation()){
        //     leaderTalon.setPosition(getCurrentAngle().getRotations(), 0.002);
        // }
        
        // inputs.firstMotorConnected =
        //     BaseStatusSignal.refreshAll(
        //             armInternalPositionRotations,
        //             armVelocityRps,
        //             armAppliedVoltage.get(0),
        //             armOutputCurrent.get(0),
        //             armTorqueCurrent.get(0),
        //             armTempCelsius.get(0))
        //         .isOK();
        // inputs.secondMotorConnected =
        //     BaseStatusSignal.refreshAll(
        //             armAppliedVoltage.get(1),
        //             armOutputCurrent.get(1),
        //             armTorqueCurrent.get(1),
        //             armTempCelsius.get(1))
        //         .isOK();

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
        TalonFXSimState lSimState = followerTalon.getSimState();
        // System.out.println(lSimState.getMotorVoltage());
        m_armSim.setInputVoltage(-lSimState.getMotorVoltage());

        m_armSim.update(0.02);

        leaderTalon.getSimState().setRawRotorPosition(-Rotation2d.fromRadians(m_armSim.getAngleRads()).getRotations()*ShooterPivotConstants.gearboxRatio);
        followerTalon.getSimState().setRawRotorPosition(Rotation2d.fromRadians(m_armSim.getAngleRads()).getRotations()*ShooterPivotConstants.gearboxRatio);

        m_canCoder.getSimState().setRawPosition(Rotation2d.fromRadians(m_armSim.getAngleRads()).getRotations());


    }

    @Override 
    public void setPivotCurrentLimit(double limit){
        leaderConfig.CurrentLimits.SupplyCurrentLimit = limit;
        leaderTalon.getConfigurator().apply(leaderConfig, 0.1);
    }

  
    
    @Override
    public Rotation2d getDesiredAngle() {
        return m_desiredAngle;
    }

    @Override
    public Rotation2d getCurrentAngle() {
        if(Robot.isSimulation()){
            return Rotation2d.fromRotations(leaderTalon.getPosition().getValueAsDouble());
        }

        return Rotation2d.fromDegrees(Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition()-absoluteEncoder.getPositionOffset()+0.5).getDegrees() % 180);
    }

    @Override
    public double getCurrentVelocity() {
        return Rotation2d.fromRotations(armVelocityRps.getValueAsDouble()).getRadians();
    }
}