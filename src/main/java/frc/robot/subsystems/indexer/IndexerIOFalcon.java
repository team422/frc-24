package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants.PivotConstants;
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
        


    public IndexerIOFalcon(int motorID,int motorID2, int initialBeamBreakID, int finalBeamBreakID) {
        m_falconFirst = new TalonFX(motorID);
        m_config = m_falconFirst.getConfigurator();
        m_slot0 = new Slot0Configs();
        m_slot0.kP = Constants.IndexerConstants.kIndexerP.get();
        m_slot0.kI = Constants.IndexerConstants.kIndexerI.get();
        m_slot0.kD = Constants.IndexerConstants.kIndexerD.get();
        // Leader motor configs
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leaderConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leaderConfig.Feedback.SensorToMechanismRatio = 1.0;
        leaderConfig.Feedback.RotorToSensorRatio = IndexerConstants.gearboxRatio;
        leaderConfig.Slot0 = m_slot0;

        m_falconFirst.getConfigurator().apply(leaderConfig);

        m_falconSecond = new TalonFX(motorID2);
        m_falconSecond.setControl(new Follower(motorID, false));
        
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_falconSecond.getConfigurator().apply(followerConfig);


        m_initialBeamBreak = new DigitalInput(initialBeamBreakID);
        m_finalBeamBreak = new DigitalInput(finalBeamBreakID);
    }


    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.curVelocity = m_falconFirst.getVelocity().getValueAsDouble();
        inputs.voltage = m_falconFirst.getMotorVoltage().getValueAsDouble();
        inputs.outputCurrent = m_falconFirst.getSupplyCurrent().getValueAsDouble();
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
            m_falconFirst.set(0);
        } else if (state == IndexerState.INTAKING) {
            m_falconFirst.setControl(new VelocityTorqueCurrentFOC(IndexerConstants.kIndexerSpeed));
            if (m_initialBeamBreak.get()) {
                RobotState.getInstance().setGamePieceLocation(GamePieceLocation.INDEXER);
            }
        } else if (state == IndexerState.INDEXING) {
            m_falconFirst.setControl(positionControl.withPosition(IndexerConstants.kIndexerLength/IndexerConstants.kRollerDiameter));
            if (m_finalBeamBreak.get()) {
                m_falconFirst.setPosition(0);
            }

        } else if (state == IndexerState.SHOOTING) {
            m_falconFirst.set(Constants.IndexerConstants.kShootingSpeed.get());
        }
    }

    @Override
    public void startIndexingPositionControl() {
        m_falconFirst.setPosition(0,0.02);
    }
}
