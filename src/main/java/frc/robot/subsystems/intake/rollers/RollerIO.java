package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;

public interface RollerIO extends LoggedIO<RollerIO.RollerInputs> {
    @AutoLog
    public static class RollerInputs {
        public double curVelocity;
        public double voltage;
        public double outputCurrent;
    }

    public void setVoltage(double voltage);

}
