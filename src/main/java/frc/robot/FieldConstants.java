package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public final static double kFieldLength = 15.98;
    public final static double kFieldWidth = 8.21;
    public final static double aprilTagWidth = Units.inchesToMeters(6.5);
    public static final AprilTagFieldLayout getAprilTags(){
        return  AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    }
}
