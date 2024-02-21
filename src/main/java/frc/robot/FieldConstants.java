package frc.robot;

import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public final static double kFieldLength = 15.98;
    public final static double kFieldWidth = 8.21;
    public final static double aprilTagWidth = Units.inchesToMeters(6.5);
    public static final AprilTagFieldLayout getAprilTags(){
        return  AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    }

    public static final HashMap<String,Pose2d> getGamePieces(){
        HashMap<String, Pose2d> gamePieces = new HashMap<String, Pose2d>();
        gamePieces.put("CloseRight", new Pose2d(Units.inchesToMeters(325), Units.inchesToMeters(160), new Rotation2d()));
        return gamePieces;
    }


}
