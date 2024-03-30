package frc.robot.subsystems.amp;

import frc.lib.advantagekit.LoggedIO;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface AmpIO extends LoggedIO<AmpIO.AmpIOInputs> {
    @AutoLog
    public class AmpIOInputs {

        double angleDegrees;
        double current;
        double voltage;
        double velocity;

    }

    public Rotation2d getPosition();

    public void setPosition(Rotation2d pos);

    public void zero();

    public void setPID(double p, double i, double d);
    
}
