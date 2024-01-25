package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.climb.ClimbIO.ClimbIOInputs;

public interface ClimbIO extends LoggedIO<ClimbIOInputs> {

    @AutoLog
    public static class ClimbIOInputs {
        public double height;
        public double velocity;
        public double outputVoltage;
        public double currentAmps;
        public boolean brake;
    }

    public void setVoltage(double voltage);

    public void setBrakeMode(boolean enabled);

}
