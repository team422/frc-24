package frc.robot;
import org.apache.commons.math3.geometry.euclidean.threed.Plane;

import com.ctre.phoenix6.configs.FeedbackConfigs;

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
import frc.lib.utils.LoggedTunableNumber;
import frc.lib.utils.TunableNumber;

public final class Constants {
    public static final boolean tuningMode = true;
    public static final double loopPeriodSecs = 0.02;

  public static final class MetaConstants {
    public static final boolean pathTuningMode = true;
  }

  public static final class Ports {
    public static final int pigeonPort = 22;

    public static final int intakeMotorPort = 43;

    public static final int wristMotorPort = 5;
    public static final int wristThroughborePort = 4;

    public static final int elevatorLeaderMotorPort = 1;
    public static final int elevatorFollowerMotorPort = 2;

    public static final int elevatorThroughBoreEncoderPortA = 1;
    public static final int elevatorThroughBoreEncoderPortB = 0;

    // Left Front Ports
    public static final int leftFrontDrivingMotorPort = 12;
    public static final int leftFrontTurningMotorPort = 7;
    public static final int leftFrontCanCoderPort = 18;

    // Right Front Ports
    public static final int rightFrontDriveMotorPort = 6;
    public static final int rightFrontTurningMotorPort = 39;
    public static final int rightFrontCanCoderPort = 17;

    // Left Rear Ports
    public static final int leftRearDriveMotorPort = 9;
    public static final int leftRearTurningMotorPort = 11;
    public static final int leftRearCanCoderPort = 16;

    // Right Rear Ports
    public static final int rightRearDriveMotorPort = 3;
    public static final int rightRearTurningMotorPort = 8;
    public static final int rightRearCanCoderPort = 15;
  }



  public static final class RobotConstants {
    public static final double trackWidth = Units.inchesToMeters(23.5);
    public static final boolean AScopeLogging = true;
  }

  public static final class IntakeConstants {
    public static final double intakeSpeed = 0.5;
    public static final double intakeSpeedToMPS = 1 * Units.inchesToMeters(1.5) * Math.PI;
    public static final double pivotGearRatio = 1;

  public static final class IntakeConstants {
    public static final double kIntakeVoltage = 12;
  }

  public static final class ShooterConstants {

    public static final class FlywheelConstants {
      public static final double kFlywheelGearRatio = 1/1.5;
      public static final double kFlywheelDiameter = Units.inchesToMeters(4);
      public static final TunableNumber kFlywheelP = new TunableNumber("Flywheel P", 0.0006, "Shooter");
      public static final TunableNumber kFlywheelI = new TunableNumber("Flywheel I", 0.0, "Shooter");
      public static final TunableNumber kFlywheelD = new TunableNumber("Flywheel D", 0.05, "Shooter");
      public static final TunableNumber kFlywheelKS = new TunableNumber("Flywheel KS", .33329, "Shooter");
      public static final TunableNumber kFlywheelKV = new TunableNumber("Flywheel KV", 0.00083, "Shooter");
      public static final TunableNumber kFlywheelKA = new TunableNumber("Flywheel KA", 0.0, "Shooter");

    }

    public static final class PivotConstants {
      // Real constants
      public static final double gearboxRatio = 46.722;
      public static final TunableNumber kPivotP = new TunableNumber("Pivot P", 0.0006, "Shooter");
      public static final TunableNumber kPivotI = new TunableNumber("Pivot I", 0.0, "Shooter");
      public static final TunableNumber kPivotD = new TunableNumber("Pivot D", 0.05, "Shooter");
      public static final TunableNumber kPivotkS = new TunableNumber("Pivot KS", .33329, "Shooter");
      public static final TunableNumber kPivotkV = new TunableNumber("Pivot KV", 0.00083, "Shooter");
      public static final TunableNumber kPivotkA = new TunableNumber("Pivot KA", 0.0, "Shooter");
      public static final TunableNumber kPivotkG = new TunableNumber("Pivot KG", 0.0, "Shooter"); 


      
      public static final Rotation2d maxAngle = Rotation2d.fromDegrees(86); 
      public static final Rotation2d minAngle = Rotation2d.fromDegrees(15);
      public static final Rotation2d homeAngle = Rotation2d.fromDegrees(35);

      public static final double maxSpeed = 1.0;
      public static final double maxAcceleration = 1.0;

      // SIM
      public static final Rotation2d simOffset = Rotation2d.fromDegrees(35);

    }
  }

  public static final class DriveConstants {
    public static final Integer kId = 1;
    public static final Integer kOdometryFrequency = 100;
    public static final double kMaxSpeedMetersPerSecond = 13.0;
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
    public static final Pose2d startPose = new Pose2d(Units.feetToMeters(3), Units.feetToMeters(3), new Rotation2d());
    public static final Rotation2d pitchAngle = Rotation2d.fromDegrees(-1.17);

    public static final boolean useTorqueCurrentFOC = false;
    public static final boolean useMotionMagic = false;
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
    public static final TunableNumber kTurningD = new TunableNumber("Turning D", 0.00 , "Drive");

    public static final TunableNumber kDriveKS = new TunableNumber("Drive KS", 1.0, "Drive");
    public static final TunableNumber kDriveKV = new TunableNumber("Drive KV", 3.0, "Drive");
    public static final TunableNumber kDriveKA = new TunableNumber("Drive KA", 1.5, "Drive");

    public static final TunableNumber kTurningPSim = new TunableNumber("TurningP Sim", 4.5, "Drive");
    public static final TunableNumber kTurningISim = new TunableNumber("Turning I Sim", 0.00, "Drive");
    public static final TunableNumber kTurningDSim = new TunableNumber("Turning D Sim", 0.005, "Drive");

    public static final TunableNumber kStartAccel = new TunableNumber("Start Accel", 4.5, "Drive");
    public static final TunableNumber kAccelDropoff = new TunableNumber("Accel Dropoff", -1, "Drive");

    public static final TunableNumber kStartAccelSim = new TunableNumber("Start Accel Sim", 4.5, "Drive");
    public static final TunableNumber kAccelDropoffSim = new TunableNumber("Accel Dropoff Sim", -1, "Drive");

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // 0.09398; // 3.7 in

    public static final double kDriveGearRatio = 5.35714286; // OLD 6.75
    public static final double kDriveConversionFactor = 1/((kDriveGearRatio/ ((Units.inchesToMeters(4) * Math.PI) )));
    public static final double kDriveConversionFactorSim = 1/(Units.inchesToMeters(4)*Math.PI);
    // public static final double kDriveConversionFactor = 1 / 22.0409;

    public static final double kTurnPositionConversionFactor = 21.428;




    // Simulation Constants
    public static final double kDriveJ = 0.025;
    public static final double kTurningJ = 0.004;

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

  public static final class IndexerConstants {
    public static final double kIndexerSpeed = 0.5;
    public static final double kRollerDiameter = Units.inchesToMeters(2);    

    public static final LoggedTunableNumber kIndexerP = new LoggedTunableNumber("Indexer P", 0.0006, "Indexer");
    public static final LoggedTunableNumber kIndexerI = new LoggedTunableNumber("Indexer I", 0.0, "Indexer");
    public static final LoggedTunableNumber kIndexerD = new LoggedTunableNumber("Indexer D", 0.05, "Indexer");
    

  }

  public static final class Vision {
    public static final class AprilTagVisionConstants {
    public static final Pose3d kRightCameraTransform = new Pose3d(new Translation3d(
        Units.inchesToMeters(6.366), Units.inchesToMeters(-8.055), Units.inchesToMeters(27.269)),
        new Rotation3d(0, Units.degreesToRadians(-10), Units.degreesToRadians(-15)));
      public static final Transform3d kleftCameraTransform = new Pose3d(new Translation3d(
          Units.inchesToMeters(6.366), Units.inchesToMeters(8.055), Units.inchesToMeters(26.269)),
          new Rotation3d(0, Units.degreesToRadians(10), Units.degreesToRadians(15))).minus(new Pose3d());

    }

    public static final class ObjectDetection {

    }

    public static final class ClimbConstants {
      public static final double kClimbGearRatio = 10.5;
      public static final double kClimbBarrierDiameter = Units.inchesToMeters(1.5);

      public static final double kMaxVelocity = 15;
      public static final double kMaxAcceleration = 20;
      public static final double kMinHeight = 0.0;
      public static final double kMaxHeight = 3.0;

      public static final TunableNumber kClimbP = new TunableNumber("Climb P", 10, "Climb");
      public static final TunableNumber kClimbI = new TunableNumber("Climb I", 0.05, "Climb");
      public static final TunableNumber kClimbD = new TunableNumber("Climb D", 1, "Climb");

      public static final TunableNumber kClimbUpSpeed = new TunableNumber("Climber Up Speed", 0.1, "Climb");
      public static final TunableNumber kClimbDownSpeed = new TunableNumber("Climber Down Speed", 0.3, "Climb");
    }

    public static final class OIConstants {
      public static final int kDriverLeftDriveStickPort = 0;
      public static final int kDriverRightDriveStickPort = 1;

      public static final double kDualFlightStickDeadzone = 0.3;
    }
  }

  public static final class TrajectoryGenerationManager {
    public static final String kTrajectoryTableName = "Trajectories";
  }
}
