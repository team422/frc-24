package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants.ShooterPivotConstants;
import frc.robot.RobotState.GamePieceLocation;
import frc.robot.subsystems.indexer.Indexer.IndexerState;
import frc.robot.utils.NoteVisualizer;

public class IndexerIOFalcon implements IndexerIO {


    
    TalonFX m_falconFirst;
    TalonFXConfigurator m_config;
    Slot0Configs m_slot0;

    DCMotorSim m_motorSim;
    // TODO: change
    private final double simGearing = 1;
    private final double simJkGMetersSquared = 1;

    double thresholdFirst = 0;
    double thresholdSecond = 0;

    TalonFX m_falconSecond;
private final PositionTorqueCurrentFOC positionControl =
        new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

    double distanceToFront = -1;
    double autoTimerShot = -1;

    DigitalInput m_initialBeamBreak;
    DigitalInput m_finalBeamBreak;

    DIOSim m_initialBeamBreakSim;
    DIOSim m_finalBeamBreakSim;

    double m_timeout = -1;

    private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0.0);
        
    static enum BeambreakSimState {
        IDLE,
        TRIGGERED,
        PIECE_IN_PLACE,
        SHOOTING
    }

    BeambreakSimState m_beamBreakSimState = BeambreakSimState.IDLE;

    public IndexerIOFalcon(int motorID,int motorID2, int initialBeamBreakID, int finalBeamBreakID) {
        m_falconFirst = new TalonFX(motorID);
        m_config = m_falconFirst.getConfigurator();
        m_slot0 = new Slot0Configs();
        m_slot0.kP = Constants.IndexerConstants.kIndexerP.get();
        m_slot0.kI = Constants.IndexerConstants.kIndexerI.get();
        m_slot0.kD = Constants.IndexerConstants.kIndexerD.get();
        // Leader motor configs
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leaderConfig.CurrentLimits.SupplyCurrentLimit = 15.0;
        leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leaderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leaderConfig.Feedback.SensorToMechanismRatio = 1.0;
        leaderConfig.Feedback.RotorToSensorRatio = IndexerConstants.gearboxRatio;
        leaderConfig.Slot0 = m_slot0;

        m_config.apply(leaderConfig);

        m_falconSecond = new TalonFX(motorID2);
        // m_falconSecond.setControl(new Follower(motorID, false));
        
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        followerConfig.Slot0 = m_slot0;

        m_falconSecond.getConfigurator().apply(followerConfig);


        m_initialBeamBreak = new DigitalInput(initialBeamBreakID);
        m_finalBeamBreak = new DigitalInput(finalBeamBreakID);

        if (RobotBase.isSimulation()) {
            m_motorSim = new DCMotorSim(DCMotor.getKrakenX60(2), simGearing, simJkGMetersSquared);
            m_initialBeamBreakSim = new DIOSim(initialBeamBreakID);
            m_finalBeamBreakSim = new DIOSim(finalBeamBreakID);
        }
    }


    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.curVelocity = m_falconFirst.getVelocity().getValueAsDouble();
        inputs.voltage = m_falconFirst.getMotorVoltage().getValueAsDouble();
        inputs.beamBreakOneBroken = m_initialBeamBreak.get();
        inputs.outputCurrent = m_falconFirst.getSupplyCurrent().getValueAsDouble();
        inputs.beamBreakTwoBroken = m_finalBeamBreak.get();

        if (RobotBase.isSimulation()) {
            simulationPeriodic();
        }
        
    }

    public void simulationPeriodic() {
        if (m_timeout > 0 && m_timeout < Timer.getFPGATimestamp()) {
            if (m_beamBreakSimState == BeambreakSimState.TRIGGERED) {
                m_finalBeamBreakSim.setValue(true);
                m_beamBreakSimState = BeambreakSimState.PIECE_IN_PLACE;
                m_timeout = Timer.getFPGATimestamp() + 0.7;
            } else if (m_beamBreakSimState == BeambreakSimState.PIECE_IN_PLACE) {
                m_initialBeamBreakSim.setValue(false);
                m_beamBreakSimState = BeambreakSimState.SHOOTING;
                m_timeout = Timer.getFPGATimestamp() + 0.7;
            } else if (m_beamBreakSimState == BeambreakSimState.SHOOTING) {
                m_finalBeamBreakSim.setValue(false);
                m_beamBreakSimState = BeambreakSimState.IDLE;
                m_timeout = -1;
            }
        }
        m_motorSim.setInputVoltage(m_falconFirst.getMotorVoltage().getValueAsDouble());
        m_motorSim.update(0.02);
    }

    @Override
    public boolean inContactWithGamePiece() {
        return m_initialBeamBreak.get() || m_finalBeamBreak.get();
    }


    @Override
    public boolean gamePieceReady() {
        return m_finalBeamBreak.get();
    }


    @Override
    public void manageState(IndexerState state) {

        if(!m_initialBeamBreak.get()){
            Logger.recordOutput("Rumble", 0.1);
            RobotState.getInstance().setDriverRumble(0.1,RumbleType.kBothRumble);
        }else{
            RobotState.getInstance().setDriverRumble(0.0, RumbleType.kBothRumble);
        }

        if (state == IndexerState.IDLE) {
            m_falconFirst.setControl(new NeutralOut());
            m_falconSecond.setControl(new NeutralOut());
        } else if (state == IndexerState.INTAKING) {
            m_falconFirst.setControl(velocityControl.withVelocity(IndexerConstants.kIndexerSpeed));
            m_falconSecond.setControl(velocityControl.withVelocity(0));
            m_falconSecond.setControl(velocityControl.withVelocity(0));
            if (!m_initialBeamBreak.get() && !edu.wpi.first.wpilibj.RobotState.isAutonomous() && !edu.wpi.first.wpilibj.RobotState.isTeleop()) {
                RobotState.getInstance().setGamePieceLocation(GamePieceLocation.INDEXER);
                
                Logger.recordOutput("Autostow noteindexer", Timer.getFPGATimestamp());
            }
        } else if (state == IndexerState.INDEXING) {
            
            m_falconSecond.setControl(velocityControl.withVelocity(0));
            if (!m_finalBeamBreak.get()) {
                m_falconFirst.setControl(new NeutralOut());
            }else{
                // m_falconFirst.setControl(new NeutralOut());
                m_falconFirst.setControl(velocityControl.withVelocity(IndexerConstants.kIndexerSpeed));
            }

        } else if (state == IndexerState.SHOOTING) {
            if (edu.wpi.first.wpilibj.RobotState.isAutonomous()){
                if (autoTimerShot == -1){
                    autoTimerShot = Timer.getFPGATimestamp() + 1.25;
                }
                m_falconFirst.setControl(velocityControl.withVelocity(IndexerConstants.kIndexerSpeedAuto));
                m_falconSecond.setControl(velocityControl.withVelocity(IndexerConstants.kIndexerSpeedAuto));
            }else{
                m_falconFirst.setControl(velocityControl.withVelocity(IndexerConstants.kIndexerShootingSpeed));
                m_falconSecond.setControl(velocityControl.withVelocity(IndexerConstants.kIndexerShootingSpeed));
            }
            if (m_finalBeamBreak.get()) {
                RobotState.getInstance().setGamePieceLocation(GamePieceLocation.SHOOTER);
                RobotState.getInstance().setIndexer(IndexerState.IDLE);
                
            }
            
            if(edu.wpi.first.wpilibj.RobotState.isAutonomous()){
                if(autoTimerShot < Timer.getFPGATimestamp()){
                    RobotState.getInstance().setGamePieceLocation(GamePieceLocation.SHOOTER);
                    autoTimerShot = -1;
                }
        }

        } else if (state == IndexerState.BACKTOINTAKE){
            m_falconFirst.setControl(velocityControl.withVelocity(-IndexerConstants.kIndexerSpeed/2));
            m_falconSecond.setControl(velocityControl.withVelocity(-IndexerConstants.kIndexerSpeed/2));

        }
    }

    public boolean getFirstBeamBreak(){
        if(!m_initialBeamBreak.get()){
            thresholdFirst +=1;
            if(thresholdFirst > 3){
                thresholdFirst = 0;
                return false;
            }
            return true;
        }else {
            thresholdFirst = 0;
            return true;

        }
    }

    @Override
    public void startIndexingPositionControl() {
        // m_falconFirst.setPosition(0,0.02);
    }

    @Override
    public void setSpeed(double speed) {
        m_falconFirst.setControl(velocityControl.withVelocity(speed));

    }

    @Override
    public double getVoltage() {
        return m_falconFirst.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setInitalBeamBreak(boolean broken) {
        m_initialBeamBreakSim.setValue(broken);
    }

    @Override
    public void setFinalBeamBreak(boolean broken) {
        m_finalBeamBreakSim.setValue(broken);
    }

    @Override
    public void gamepiece() {
        m_timeout = Timer.getFPGATimestamp() + 0.7;
        m_beamBreakSimState = BeambreakSimState.TRIGGERED;
        m_initialBeamBreakSim.setValue(true);
    }
}
