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

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants.ShooterPivotConstants;
import frc.robot.RobotState.GamePieceLocation;
import frc.robot.subsystems.indexer.Indexer.IndexerState;

public class IndexerIOFalcon implements IndexerIO {


    
    TalonFX m_falconFirst;
    TalonFXConfigurator m_config;
    Slot0Configs m_slot0;

    TalonFX m_falconSecond;
private final PositionTorqueCurrentFOC positionControl =
        new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

    double distanceToFront = -1;

    DigitalInput m_initialBeamBreak;
    DigitalInput m_finalBeamBreak;

    private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0.0);
        


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
    }


    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.curVelocity = m_falconFirst.getVelocity().getValueAsDouble();
        inputs.voltage = m_falconFirst.getMotorVoltage().getValueAsDouble();
        inputs.beamBreakOneBroken = m_initialBeamBreak.get();
        inputs.outputCurrent = m_falconFirst.getSupplyCurrent().getValueAsDouble();
        inputs.beamBreakTwoBroken = m_finalBeamBreak.get();
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
        if (state == IndexerState.IDLE) {
            m_falconFirst.setControl(new NeutralOut());
            m_falconSecond.setControl(new NeutralOut());
        } else if (state == IndexerState.INTAKING) {
            m_falconFirst.setControl(velocityControl.withVelocity(IndexerConstants.kIndexerSpeed));
            m_falconSecond.setControl(velocityControl.withVelocity(0));
            m_falconSecond.setControl(velocityControl.withVelocity(0));
            if (!m_initialBeamBreak.get()) {
                RobotState.getInstance().setGamePieceLocation(GamePieceLocation.INDEXER);
                Logger.recordOutput("IS SETTING", true);
            }
        } else if (state == IndexerState.INDEXING) {
            
            m_falconSecond.setControl(velocityControl.withVelocity(0));
            if (!m_finalBeamBreak.get()) {
                m_falconFirst.setControl(new NeutralOut());
            }else{
                m_falconFirst.setControl(velocityControl.withVelocity(IndexerConstants.kIndexerSpeed));
            }

        } else if (state == IndexerState.SHOOTING) {
            m_falconFirst.setControl(velocityControl.withVelocity(IndexerConstants.kIndexerSpeed));
            m_falconSecond.setControl(velocityControl.withVelocity(IndexerConstants.kIndexerSpeed));
            if (m_finalBeamBreak.get() && m_initialBeamBreak.get()) {
                RobotState.getInstance().setGamePieceLocation(GamePieceLocation.SHOOTER);

            }

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
}
