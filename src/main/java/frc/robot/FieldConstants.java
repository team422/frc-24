package frc.robot;

import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.AllianceFlipUtil;

public class FieldConstants {

  public static final class Source {
    public static final Translation2d source = new Translation2d(15.696, 0.701);

  }

  public static final Translation3d topRightSpeaker =
        new Translation3d(
            Units.inchesToMeters(0.0),
            Units.inchesToMeters(238.815),
            Units.inchesToMeters(83.091));

    public static final Translation3d topLeftSpeaker =
        new Translation3d(
            Units.inchesToMeters(18.055),
            Units.inchesToMeters(197.765),
            Units.inchesToMeters(83.091));

    public static final Translation3d bottomRightSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
    public static final Translation3d bottomLeftSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));

    /** Center of the speaker opening (blue alliance) */
    public static final Translation3d centerSpeakerOpening =
        bottomLeftSpeaker.interpolate(topRightSpeaker, 0.5);
    public final static double kFieldLength = 15.98;
    public static final double fieldLength = Units.inchesToMeters(651.223);
    public static final double fieldWidth = Units.inchesToMeters(323.277);
    public static final double wingX = Units.inchesToMeters(229.201);
    public static final double podiumX = Units.inchesToMeters(126.75);
    public static final double startingLineX = Units.inchesToMeters(74.111);
    public final static double kFieldWidth = 8.21;

    public final static double aprilTagWidth = Units.inchesToMeters(6.5);
    public final static AprilTagFieldLayout field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static final Pose2d kAmpBlue = new Pose2d(1.749, 7.82, Rotation2d.fromDegrees(90));
    public static final Pose2d kDailedShot = new Pose2d(2.95, 4.08, new Rotation2d(145.00));
    public static final Translation2d kCorner = new Translation2d(0, 7.82);
    public static final Translation2d kFeederAim = new Translation2d(1,6.82);
    public static final Translation2d kSourceMidShot = new Translation2d(8.04,2);
    


    public static final AprilTagFieldLayout getAprilTags(){
        return field;
    }

    public static final class StagingLocations {
    public static final double centerlineX = fieldLength / 2.0;

    // need to update
    public static final double centerlineFirstY = Units.inchesToMeters(29.638);
    public static final double centerlineSeparationY = Units.inchesToMeters(66);
    public static final double spikeX = Units.inchesToMeters(114);
    // need
    public static final double spikeFirstY = Units.inchesToMeters(161.638);
    public static final double spikeSeparationY = Units.inchesToMeters(57);

    public static final Translation2d[] centerlineTranslations = new Translation2d[5];
    public static final Translation2d[] spikeTranslations = new Translation2d[3];

    static {
      for (int i = 0; i < centerlineTranslations.length; i++) {
        centerlineTranslations[i] =
            new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
      }
    }

    static {
      for (int i = 0; i < spikeTranslations.length; i++) {
        spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
      }
    }
  }


    public static final HashMap<String,Pose2d> getGamePieces(){
        HashMap<String, Pose2d> gamePieces = new HashMap<String, Pose2d>();
        // gamePieces.put("CloseRight", new Pose2d(Units.inchesToMeters(325), Units.inchesToMeters(160), new Rotation2d()));
        for (int i = FieldConstants.StagingLocations.spikeTranslations.length - 1; i >= 0; i--) {
            gamePieces.put(i+"Spike",new Pose2d(AllianceFlipUtil.apply(FieldConstants.StagingLocations.spikeTranslations[i]), new Rotation2d()));
        }
        for (int i = FieldConstants.StagingLocations.centerlineTranslations.length - 1; i >= 0; i--) {
            gamePieces.put(i+"Centerline",new Pose2d(AllianceFlipUtil.apply(FieldConstants.StagingLocations.centerlineTranslations[i]), new Rotation2d()));
        }

        return gamePieces;
    }


}
