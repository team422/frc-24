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
    }
    

    public void setDesiredAngle(Rotation2d angle);

    public void updateInputs(PivotIOInputs inputs);
}
