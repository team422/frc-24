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
        public double feederVoltage;
        public double kickerVoltage;
        public double distanceToFront;
        public double curFeederVelocity;
        public double curKickerVelocity;
        public double feederOutputCurrent;
        public double kickerOutputCurrent;
        public double desiredFeederSpeeds;
        public double desiredKickerSpeeds;
        
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
