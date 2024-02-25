package frc.robot;

import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public final static double kFieldLength = 15.98;
    public static final double fieldLength = Units.inchesToMeters(651.223);
    public static final double fieldWidth = Units.inchesToMeters(323.277);
    public static final double wingX = Units.inchesToMeters(229.201);
    public static final double podiumX = Units.inchesToMeters(126.75);
    public static final double startingLineX = Units.inchesToMeters(74.111);
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
