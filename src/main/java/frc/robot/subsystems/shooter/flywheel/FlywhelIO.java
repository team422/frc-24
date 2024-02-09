package frc.robot.subsystems.shooter.flywheel;
import org.littletonrobotics.junction.AutoLog;

public interface FlywhelIO {

  @AutoLog
  public class FlywheelInputs {
    public double velocityMetersPerSec;
    public double velocityRadPerSec;
  }

  public double getVelocityMetersPerSec();

  public double getVelocityRevPerMin();

  public double getVelocityRadPerSec();

  public void setVoltage(double voltage);

  public double getWheelLength();

  public void updateInputs(FlywheelInputsAutoLogged inputs);

}

