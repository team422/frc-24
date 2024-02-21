package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class IndexerIOFalcon implements IndexerIO {


    
    TalonFX m_falcon;
    TalonFXConfigurator m_config;
    Slot0Configs m_slot0;


    double distanceToFront = -1;

    DigitalInput m_initialBeamBreak;
    DigitalInput m_finalBeamBreak;
        

    public enum IndexerState {
        IDLE,
        INTAKING,
        INDEXING,
        SHOOTING
    }

    IndexerState m_state = IndexerState.IDLE;


    public IndexerIOFalcon(int motorID, int initialBeamBreakID, int finalBeamBreakID) {
        m_falcon = new TalonFX(motorID);
        m_config = m_falcon.getConfigurator();
        m_slot0 = new Slot0Configs();
        m_slot0.kP = Constants.IndexerConstants.kIndexerP.get();
        m_slot0.kI = Constants.IndexerConstants.kIndexerI.get();
        m_slot0.kD = Constants.IndexerConstants.kIndexerD.get();


        m_initialBeamBreak = new DigitalInput(initialBeamBreakID);
        m_finalBeamBreak = new DigitalInput(finalBeamBreakID);
    }


    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.curVelocity = m_falcon.getVelocity().getValueAsDouble();
        inputs.voltage = m_falcon.getMotorVoltage().getValueAsDouble();
        inputs.outputCurrent = m_falcon.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public boolean inContactWithGamePiece() {
        return m_initialBeamBreak.get() || m_finalBeamBreak.get();
    }

    @Override
    public boolean gamePieceReady() {
        return m_finalBeamBreak.get();
    }
}
