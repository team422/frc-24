package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;

public interface IndexerIO extends LoggedIO<IndexerIO.IndexerIOInputs> {
    @AutoLog
    public class IndexerIOInputs {
        public boolean inContactWithGamePiece;
        public boolean gamePieceReady;
        public boolean beamBreakOneBroken;
        public boolean beamBreakTwoBroken;
        public double voltage;
        public double current;
        public double distanceToFront;
        public double curVelocity;
        public double outputCurrent;
        public double desiredSpeeds;

    }

    public boolean inContactWithGamePiece();

    public boolean gamePieceReady();

    public void manageState(Indexer.IndexerState state);

    public void startIndexingPositionControl();


    public void setSpeed(double speed);

    public double getVoltage();

    public void setInitalBeamBreak(boolean broken);

    public void setFinalBeamBreak(boolean broken);

    public void gamepiece();
    
}
