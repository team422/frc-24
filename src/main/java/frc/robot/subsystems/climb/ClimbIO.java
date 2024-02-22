package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;

public interface ClimbIO extends LoggedIO<ClimbIO.ClimbIOInputs> {
    @AutoLog
    public class ClimbIOInputs {
        public double voltage;
        public double current;
        public double curVelocity;
        public double outputCurrent;
        public double desiredSpeeds;
        public double height;

    }

    public void lockServos();

    public void unlockServos();

    public void updateInputs(ClimbIOInputs inputs);
    
}
