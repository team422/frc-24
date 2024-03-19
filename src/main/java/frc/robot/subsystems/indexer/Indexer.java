package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import frc.lib.hardwareprofiler.ProfiledSubsystem;
import frc.robot.Robot;
import frc.robot.utils.NoteVisualizer;

public class Indexer extends ProfiledSubsystem {
    private IndexerIO io;
    private IndexerIOInputsAutoLogged m_inputs;

    public enum IndexerState {
        IDLE,
        INTAKING,
        INDEXING, 
        SHOOTING,
        BACKTOINTAKE
    }

    IndexerState m_state = IndexerState.IDLE;
    // IDLE means that there is no piece in the indexer so no beam breaks are triggered
    // INTAKING means that the initial beam break is triggered but the final beam break is not
    // INDEXING means that it has been moved to the final beam break
    // SHOOTING means that the indexer is moving the game piece to the shooter


    public Indexer(IndexerIO io) {
        this.io = io;
        m_inputs = new IndexerIOInputsAutoLogged();
    }

    public void periodic() {
        io.updateInputs(m_inputs);
        io.manageState(m_state);
        Logger.processInputs("Indexer",m_inputs);
    }

    public boolean inContactWithGamePiece() {
        return io.inContactWithGamePiece();
    }

    public boolean gamePieceReady() {
        return io.gamePieceReady();
    }


    public void setManualSpeed(double speed) {
        io.setSpeed(speed);
    }

    public void setState(IndexerState state) {
        m_state = state;
        if (m_state == IndexerState.SHOOTING){
            if (Robot.isSimulation()){
              NoteVisualizer.shoot().schedule();;
            }
        }
        if (m_state == IndexerState.INDEXING) {
            io.startIndexingPositionControl();
        }
    }
}
