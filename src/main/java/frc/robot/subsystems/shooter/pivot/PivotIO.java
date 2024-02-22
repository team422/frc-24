package frc.robot.subsystems.shooter.pivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.advantagekit.LoggedIO;

/**
 * PivotIO
 */
public interface PivotIO extends LoggedIO<PivotIO.PivotIOInputs>{
    

    @AutoLog
    public class PivotIOInputs {
        public double curAngle;
        public double desiredAngle;
        public double curSpeed;
        public double voltage;

        public boolean firstMotorConnected = true;
        public boolean secondMotorConnected = true;
    
        public double armPositionRads = 0.0;
        public double armEncoderPositionRads = 0.0;
        public double armAbsoluteEncoderPositionRads = 0.0;
        public double armVelocityRadsPerSec = 0.0;
        public double[] armAppliedVolts = new double[] {};
        public double[] armCurrentAmps = new double[] {};
        public double[] armTorqueCurrentAmps = new double[] {};
        public double[] armTempCelcius = new double[] {};
        public boolean absoluteEncoderConnected = true;

    }
    

    public void runSetpoint(Rotation2d angle, double feedforward);

    public void updateInputs(PivotIOInputs inputs);

    public Rotation2d getDesiredAngle();

    public Rotation2d getCurrentAngle();
}
