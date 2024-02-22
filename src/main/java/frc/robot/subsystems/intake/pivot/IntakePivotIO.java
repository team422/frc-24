package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.advantagekit.LoggedIO;

public interface IntakePivotIO extends LoggedIO<IntakePivotIO.IntakePivotIOInputs>{
    @AutoLog
    public class IntakePivotIOInputs {
        public double curAngle;
        public double desiredAngle;
        public double curSpeed;
        public double voltage;
        public double desiredVoltage;
    }

    public void setDesiredAngle(Rotation2d angle);

    public void updateInputs(IntakePivotIOInputs inputs);

    public Rotation2d getAngle();

    
}
