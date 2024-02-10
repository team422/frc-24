package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.climb.ClimbIO.ClimbInputs;

public interface ClimbIO extends LoggedIO<ClimbInputs> {

    @AutoLog
    public static class ClimbInputs {
        public double height;
        public double velocity;
        public double outputVoltage;
        public double currentAmps;
        public boolean brake;
    }

    public void setVoltage(double voltage);

    public void setBrakeMode(boolean enabled);

}
