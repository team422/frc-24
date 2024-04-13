package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;

public interface WristIO {
    public void setVoltage(double voltage);

    public Rotation2d getAngle();
}
