package frc.robot.subsystems.shooter.flywheel;
import org.littletonrobotics.junction.AutoLog;
import frc.lib.advantagekit.LoggedIO;

public interface flywheelIO {

  @AutoLog
  public class FlywheelInputs {
    public double velocityMetersPerSec;
    public double velocityRadPerSec;
  }

  public double getVelocityMetersPerSec();

  public double getVelocityRevPerMin();

  public double getVelocityRadPerSec();

  public void setVoltage(double voltage);

}

