package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;

public interface RollerIO extends LoggedIO<RollerIO.RollerIOInputs> {
    @AutoLog
    public static class RollerIOInputs {
        public double curVelocity;
        public double voltage;
        public double outputCurrent;
        public double desiredSpeeds;
        public double acceleration;
        public double currentDelta;
    }

    public void setVoltage(double voltage);

   

}
