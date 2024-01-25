package frc.robot;
import org.apache.commons.math3.geometry.euclidean.threed.Plane;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.utils.CustomHolmonomicDrive;
import frc.lib.utils.TunableNumber;

public final class Constants {
  public static final boolean tuningMode = true;

  public static final class MetaConstants {
    public static final boolean pathTuningMode = true;
  }

  public static final class RobotConstants {
    public static final double trackWidth = Units.inchesToMeters(23.5);
    public static final boolean AScopeLogging = true;
  }

  public static final class IntakeConstants {
    public static final double kIntakeVoltage = 12;
  }

  public static final class ShooterConstants {
    public static final class PivotConstants {
      // Real constants
      public static final double gearboxRatio = 46.722;
      public static final Rotation2d maxAngle = Rotation2d.fromDegrees(86);
      public static final Rotation2d minAngle = Rotation2d.fromDegrees(15);
      public static final Rotation2d homeAngle = Rotation2d.fromDegrees(35);

      // SIM
      public static final Rotation2d simOffset = Rotation2d.fromDegrees(35);

    }
  }

  public static final class DriveConstants {
    public static final Integer kId = 1;
    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = 3.0 * Math.PI;
    public static final double controllerDeadzone = 0.04;
    public static final double kWheelBase = Units.inchesToMeters(23);
    public static final double kTrackWidth = Units.inchesToMeters(23);
    public static Translation2d[] kModuleTranslations = {
        new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), // front left
        new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0), // front right
        new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0), // rear left
        new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0) // rear right
    };
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(kModuleTranslations);
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 3.0 * Math.PI;
    public static final double kMaxAccelMetersPerSecondSq = 3.0;
    public static final double kMaxAngularAccelRadiansPerSecondSq = 3.0 * Math.PI;

    public static final CustomHolmonomicDrive holonomicDrive = new CustomHolmonomicDrive(new PIDController(1.0, 0, 0),
        new PIDController(.08, 0, 0), new SlewRateLimiter(kMaxAccelMetersPerSecondSq),
        new SlewRateLimiter(kMaxAccelMetersPerSecondSq),
        new SlewRateLimiter(kMaxAngularAccelerationRadiansPerSecondSquared));
    public static final double[] kDriveSpeedTests = { 1.5, 2.0, 2.5, 3.0, 3.5, 4.0 };
  }

  public static final class ModuleConstants {
    public static final TunableNumber kDriveP = new TunableNumber("Drive wP", 0.1, "Drive");
    public static final TunableNumber kDriveI = new TunableNumber("Drive I", 0.0, "Drive");
    public static final TunableNumber kDriveD = new TunableNumber("Drive D", 0.00, "Drive");
    public static final TunableNumber kDriveFF = new TunableNumber("Drive FF", 2.96, "Drive");

    public static final TunableNumber kFFDriveP = new TunableNumber("FFDrive wP", 0.1, "Drive");
    public static final TunableNumber kFFDriveI = new TunableNumber("FFDrive I", 0.0, "Drive");
    public static final TunableNumber kFFDriveD = new TunableNumber("FFDrive D", 0.00, "Drive");

    public static final TunableNumber kTurningP = new TunableNumber("TurnP", .16, "Drive");
    public static final TunableNumber kTurningI = new TunableNumber("Turning I", 0.00, "Drive");
    public static final TunableNumber kTurningD = new TunableNumber("Turning D", 0.005, "Drive");

    public static final TunableNumber kDriveKS = new TunableNumber("Drive KS", 1.0, "Drive");
    public static final TunableNumber kDriveKV = new TunableNumber("Drive KV", 3.0, "Drive");
    public static final TunableNumber kDriveKA = new TunableNumber("Drive KA", 1.5, "Drive");

    public static final TunableNumber kTurningPSim = new TunableNumber("TurningP Sim", 5.5, "Drive");
    public static final TunableNumber kTurningISim = new TunableNumber("Turning I Sim", 0.00, "Drive");
    public static final TunableNumber kTurningDSim = new TunableNumber("Turning D Sim", 0.005, "Drive");

    public static final TunableNumber kStartAccel = new TunableNumber("Start Accel", 4.5, "Drive");
    public static final TunableNumber kAccelDropoff = new TunableNumber("Accel Dropoff", -1, "Drive");

    public static final TunableNumber kStartAccelSim = new TunableNumber("Start Accel Sim", 4.5, "Drive");
    public static final TunableNumber kAccelDropoffSim = new TunableNumber("Accel Dropoff Sim", -1, "Drive");

    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.85); // 0.09398; // 3.7 in

    public static final double kDriveGearRatio = 6.75;
    // public static final double kDriveConversionFactor = ((kWheelDiameterMeters *
    // Math.PI) / kDriveGearRatio);
    public static final double kDriveConversionFactor = 1 / 22.0409;

    public static final double kTurnPositionConversionFactor = 21.428;

    // Simulation Constants
    public static final double kDriveJ = 0.01;
    public static final double kTurningJ = 0.0001;

  }

  public static final class FieldConstants {
    public static final double kFieldLengthMeters = Units.feetToMeters(54.27083);
    public static final double kFieldWidthMeters = Units.feetToMeters(26.2916);
    public static final Pose2d kOppositeField = new Pose2d(kFieldLengthMeters, kFieldWidthMeters,
        Rotation2d.fromDegrees(180));

    public static final double kShooterBaseHeight = Units.inchesToMeters(79.129 + 1/8);// ONE INCH HAS BEEN ADDED BECAUSE VISUAL MODELS SHOW IT TO BE ONE INCH HIGHER, CHECK IN REAL LIFE @JAMES REMINDER
    public static final double kShooterTopHeight = Units.inchesToMeters(98.25);
    public static final double kShooterBaseLength = Units.inchesToMeters(217.9585);
    public static final Rotation2d kShooterAngle = Rotation2d.fromDegrees(14);
    public static final Translation3d kShooterCenter = new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(217.9585), Units.inchesToMeters(162.138+4.875));
    public static final double kShooterWidth = Units.inchesToMeters(41.375);
    public static final double kShooterHeight = Units.inchesToMeters(4.875 * 2);
    public static final double kShooterDepth = Units.inchesToMeters(18);
    public static final Translation3d kShooterBackLeft = new Translation3d(Units.inchesToMeters(0), kShooterBaseLength+kShooterWidth/2,kShooterBaseHeight); 
    public static final Translation3d kShooterBackRight = new Translation3d(Units.inchesToMeters(0), kShooterBaseLength-kShooterWidth/2, kShooterBaseHeight);
    public static final Translation3d kShooterFrontLeft = new Translation3d(kShooterDepth, kShooterBaseLength+kShooterWidth/2, kShooterBaseHeight + kShooterHeight/2);
    public static final Translation3d kShooterFrontRight = new Translation3d(kShooterDepth, kShooterBaseLength-kShooterWidth/2, kShooterBaseHeight+ kShooterHeight/2);
    
    // public static final Pose3d kShooterMiddle = Pose3d()
    
  }

  public static final class Vision {
    public static final class AprilTagVision {
      public static final Pose3d kRightCameraTransform = new Pose3d(new Translation3d(
          Units.inchesToMeters(6.366), Units.inchesToMeters(-8.055), Units.inchesToMeters(27.269)),
          new Rotation3d(0, Units.degreesToRadians(10), Units.degreesToRadians(-15)));
      public static final Transform3d kleftCameraTransform = new Pose3d(new Translation3d(
          Units.inchesToMeters(6.366), Units.inchesToMeters(8.055), Units.inchesToMeters(26.269)),
          new Rotation3d(0, Units.degreesToRadians(10), Units.degreesToRadians(15))).minus(new Pose3d());

    }

    public static final class ObjectDetection {

    }

    public static final class ClimbConstants {
      public static final double kClimbGearRatio = 25.0; // radians to meters, ie how many radians of rotation is one
                                                         // meter of linear motion
      public static final double kMaxVelocity = 3.0;
      public static final double kMaxAcceleration = 1.0;
      public static final double kMinHeight = 0.0;
      public static final double kMaxHeight = 3.0;

      public static final TunableNumber kClimbP = new TunableNumber("Climb P", 10.0, "Climb");
      public static final TunableNumber kClimbI = new TunableNumber("Climb I", 0.005, "Climb");
      public static final TunableNumber kClimbD = new TunableNumber("Climb D", 0.01, "Climb");
    }
  }
}
