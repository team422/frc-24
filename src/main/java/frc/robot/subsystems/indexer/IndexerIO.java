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
    
}
