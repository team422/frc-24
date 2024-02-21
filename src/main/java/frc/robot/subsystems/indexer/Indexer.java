package frc.robot.subsystems.indexer;

import frc.lib.hardwareprofiler.ProfiledSubsystem;

public class Indexer extends ProfiledSubsystem {
    private IndexerIO io;
    private IndexerIOInputsAutoLogged m_inputs;
    public Indexer(IndexerIO io) {
        this.io = io;
        m_inputs = new IndexerIOInputsAutoLogged();
    }

    public void periodic() {
        io.updateInputs(m_inputs);
    }

    public boolean inContactWithGamePiece() {
        return io.inContactWithGamePiece();
    }

    public boolean gamePieceReady() {
        return io.gamePieceReady();
    }
}
