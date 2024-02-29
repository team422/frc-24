package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;

public interface FlywheelIO extends LoggedIO<FlywheelIO.FlywheelIOInputs> {
    @AutoLog
    public class FlywheelIOInputs {
        public boolean firstMotorConnected;
        public boolean secondMotorConnected;
        public double curSpeed;
        public double desiredSpeed;
        public double voltage;
        public double PositionRads = 0.0;
        public double VelocityRpm = 0.0;
        public double AppliedVolts = 0.0;
        public double OutputCurrent = 0.0;
        public double TempCelsius = 0.0;
        public double PositionRadsLeft = 0.0;
        public double VelocityRpmLeft = 0.0;
        public double AppliedVoltsLeft = 0.0;
        public double OutputCurrentLeft = 0.0;
        public double TempCelsiusLeft = 0.0;
        

    
    }

    public void setDesiredSpeed(double speed);

    public default void setDesiredSpeedWithSpin(double speedLeft, double speedRight){
    };


    public void updateInputs(FlywheelIOInputs inputs);

    public void stop();
    
}
