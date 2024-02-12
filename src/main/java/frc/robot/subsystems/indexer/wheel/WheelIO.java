package frc.robot.subsystems.indexer.wheel;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;

public interface WheelIO extends LoggedIO<WheelIO.WheelInputs>{

    @AutoLog
    public static class WheelInputs {
        public double voltage;
        public double current;
        public double angularVelocity;
        public double linearVelocity;
        public double position;
    }

    public void setVoltage(double voltage);

    public double getVoltage();

    public double getCurrent();

    public double getAngularVelocity();

    public double getLinearVelocity();

    public double getPosition();
}
