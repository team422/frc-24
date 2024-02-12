package frc.robot.subsystems.indexer.sensor;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;

public interface SensorIO extends LoggedIO<SensorIO.SensorInputs> {
    @AutoLog
    public static class SensorInputs {
        public boolean broken;
    }

    public boolean getBroken();

    public default void toggleBroken() {
    } // only for sim 
}
