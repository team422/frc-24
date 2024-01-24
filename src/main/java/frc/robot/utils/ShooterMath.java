package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldConstants;

public class ShooterMath {

    double targetHeight = FieldConstants.kShooterBaseHeight;

    // for right now, we are going to assume the note moves linearly
    // we will need to change this later
    public static Rotation2d getShooterAngle (double distance, double height) {
        double angle = Math.atan((height - 0.5) / distance);
        return Rotation2d.fromDegrees(angle);
    }
}
