package frc.robot;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.lib.utils.CustomHolmonomicDrive;
import frc.lib.utils.LoggedTunableNumber;


public final class Constants {
    public static final boolean tuningMode = true;
    public static final boolean fullManualShooterAndPivotSpeedControls = false;
    public static final double loopPeriodSecs = 0.02;

  public static final class MetaConstants {
    public static final boolean pathTuningMode = true;
  }

  public static final class Ports {
    public static final int pigeonPort = 22;

    public static final int intakeMotorPort = 43;

    public static final int wristMotorPort = 33;
    // public static final int wristThroughborePort = 4;

    public static final int climbLeader = 25;
    public static final int climbFollower = 26;

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

    // indexer

    public static final int indexerFirst = 30;
    public static final int indexerSecond = 31;

    // Servo Ports
    public static final int servoPort = 1;
    public static final int servoPort2 = 8;

    // beambreak ports
    public static final int beamBreakPort = 1;
    public static final int beamBreakPort2 = 0;

    public static final int shooterLeft = 35;
    public static final int shooterRight = 36;

    public static final int shooterPivot = 39;
    public static final int shooterPivotFollower = 40;
  }



  public static final class RobotConstants {
    public static final double trackWidth = Units.inchesToMeters(23.5);
    public static final boolean AScopeLogging = true;
  }

  public static final class IntakeConstants {
    public static final double intakeSpeed = 0.5;
    public static final double intakeSpeedToMPS = 1 * Units.inchesToMeters(1.5) * Math.PI;
    public static final double pivotGearRatio = 36.0/16;
    public static final Rotation2d kIntakeMaxMovedAngle = Rotation2d.fromDegrees(84);
    public static final Rotation2d kIntakeMaxAngle = Rotation2d.fromDegrees(120);
    public static final Rotation2d kIntakeMinAngle = Rotation2d.fromDegrees(15);
    public static final Rotation2d kIntakeHomeAngle = Rotation2d.fromDegrees(34);

    public static final double kFlywheelTolerance = 15;

    public static final double kIntakeStowTime = 0.75; // Seconds from indexer receiving game piece to stowing the intake

    public static final LoggedTunableNumber kIntakeP = new LoggedTunableNumber("Intake P", .4, "Intake");
    public static final LoggedTunableNumber kIntakeI = new LoggedTunableNumber("Intake I", 0.0, "Intake");
    public static final LoggedTunableNumber kIntakeD = new LoggedTunableNumber("Intake D", 0.05, "Intake");


  }

  public static final class ShooterConstants {

    public static final class FlywheelConstants {
      public static final double kFlywheelGearRatio = 1/1.5;
      public static final double kFlywheelDiameter = Units.inchesToMeters(4);
      public static final LoggedTunableNumber kFlywheelP = new LoggedTunableNumber("Flywheel P", 2, "Shooter");
      public static final LoggedTunableNumber kFlywheelI = new LoggedTunableNumber("Flywheel I", 0.0, "Shooter");
      public static final LoggedTunableNumber kFlywheelD = new LoggedTunableNumber("Flywheel D", 0.05, "Shooter");
      public static final LoggedTunableNumber kFlywheelKS = new LoggedTunableNumber("Flywheel KS", .33329, "Shooter");
      public static final LoggedTunableNumber kFlywheelKV = new LoggedTunableNumber("Flywheel KV", 0.00083, "Shooter");
      public static final LoggedTunableNumber kFlywheelKA = new LoggedTunableNumber("Flywheel KA", 0.0, "Shooter");
      public static final double kMaxSpeed = 100;
      public static final double kIdleSpeed = 0;
      public static final LoggedTunableNumber kFlywheelSpeed = new LoggedTunableNumber("Flywheel Speed", 0.0, "Shooter");
    }

    public static final class ShooterPivotConstants {
      // Real constants
      public static final double gearboxRatio = 46.722222;
      public static final LoggedTunableNumber kPivotP = new LoggedTunableNumber("Pivot P", 30.0, "Shooter");

      public static final LoggedTunableNumber kPivotI = new LoggedTunableNumber("Pivot I", 0.0, "Shooter");
      public static final LoggedTunableNumber kPivotD = new LoggedTunableNumber("Pivot D", 0.05, "Shooter");
      public static final LoggedTunableNumber kPivotkS = new LoggedTunableNumber("Pivot KS", .3, "Shooter");
      public static final LoggedTunableNumber kPivotkV = new LoggedTunableNumber("Pivot KV", 0.83, "Shooter");
      public static final LoggedTunableNumber kPivotkA = new LoggedTunableNumber("Pivot KA", 0.0, "Shooter");
      public static final LoggedTunableNumber kPivotkG = new LoggedTunableNumber("Pivot KG", 0.0, "Shooter"); 

      public static final double kOffset = Rotation2d.fromDegrees(181.7-13).getRotations();

      public static final LoggedTunableNumber kShooterAngle = new LoggedTunableNumber("Shooter Angle", 0.0, "Shooter");
      // public static final double kOffset = Rotation2d.fromDegrees(0).getRotations();

      
      public static final Rotation2d maxAngle = Rotation2d.fromDegrees(77); 
      public static final Rotation2d minAngle = Rotation2d.fromDegrees(15);
      public static final Rotation2d homeAngle = Rotation2d.fromDegrees(34);

      public static final double maxSpeed = 1.0;
      public static final double maxAcceleration = 1.0;

      // SIM
      public static final Rotation2d simOffset = Rotation2d.fromDegrees(35);

    }
  }

  public static final class DriveConstants {
    public static final Integer kId = 1;
    public static final Integer kOdometryFrequency = 100;
    public static final double kMaxSpeedMetersPerSecond = 5.9;
    public static final double kMaxAngularSpeedRadiansPerSecond = 3.0 * Math.PI;
    public static final double controllerDeadzone = 0.04;
    public static final double kShootToleranceDeg = 1;
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

    public static final boolean useTorqueCurrentFOC = true;
    public static final boolean useMotionMagic = false;
    public static final Matrix<N3, N1> odometryStateStdDevs = new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.0002));
    public static final PathConstraints kAutoAlignToAmpSpeed = new PathConstraints(5.9,6,2,2);
    public static final PathConstraints kDriveToPieceSpeed = new PathConstraints(5.9,6,2,2);
  }

  public static final class ModuleConstants {
    public static final LoggedTunableNumber kDriveP = new LoggedTunableNumber("Drive wP", 2.0, "Drive");
    public static final LoggedTunableNumber kDriveI = new LoggedTunableNumber("Drive I", 0.0, "Drive");
    public static final LoggedTunableNumber kDriveD = new LoggedTunableNumber("Drive D", 0.00, "Drive");
    public static final LoggedTunableNumber kDriveFF = new LoggedTunableNumber("Drive FF", 2.96, "Drive");

    public static final LoggedTunableNumber kffkS = new LoggedTunableNumber("FF KS", 0.0, "Drive");
    public static final LoggedTunableNumber kffkV = new LoggedTunableNumber("FF KV", 0.0, "Drive");

    public static final LoggedTunableNumber kFFDriveP = new LoggedTunableNumber("FFDrive wP", 0.1, "Drive");
    public static final LoggedTunableNumber kFFDriveI = new LoggedTunableNumber("FFDrive I", 0.0, "Drive");
    public static final LoggedTunableNumber kFFDriveD = new LoggedTunableNumber("FFDrive D", 0.00, "Drive");

    public static final LoggedTunableNumber kTurningP = new LoggedTunableNumber("TurnP", 1.5, "Drive");
    public static final LoggedTunableNumber kTurningI = new LoggedTunableNumber("Turning I", 0.00, "Drive");
    public static final LoggedTunableNumber kTurningD = new LoggedTunableNumber("Turning D", 0.00 , "Drive");

    public static final LoggedTunableNumber kDriveKS = new LoggedTunableNumber("Drive KS", 1.0, "Drive");
    public static final LoggedTunableNumber kDriveKV = new LoggedTunableNumber("Drive KV", 3.0, "Drive");
    public static final LoggedTunableNumber kDriveKA = new LoggedTunableNumber("Drive KA", 1.5, "Drive");

    public static final LoggedTunableNumber kTurningPSim = new LoggedTunableNumber("TurningP Sim", 4.5, "Drive");
    public static final LoggedTunableNumber kTurningISim = new LoggedTunableNumber("Turning I Sim", 0.00, "Drive");
    public static final LoggedTunableNumber kTurningDSim = new LoggedTunableNumber("Turning D Sim", 0.005, "Drive");

    public static final LoggedTunableNumber kStartAccel = new LoggedTunableNumber("Start Accel", 4.5, "Drive");
    public static final LoggedTunableNumber kAccelDropoff = new LoggedTunableNumber("Accel Dropoff", -1, "Drive");

    public static final LoggedTunableNumber kStartAccelSim = new LoggedTunableNumber("Start Accel Sim", 4.5, "Drive");
    public static final LoggedTunableNumber kAccelDropoffSim = new LoggedTunableNumber("Accel Dropoff Sim", -1, "Drive");

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // 0.09398; // 3.7 in

    public static final double kDriveGearRatio = 5.35714286; // OLD 6.75
    public static final double kDriveConversionFactor = ((kDriveGearRatio/ ((Units.inchesToMeters(4) * Math.PI) )));
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
    public static final Translation3d kShooterCenter = new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(217.9585), Units.inchesToMeters(80));
    public static final Translation3d kShooterLeftCenter = kShooterCenter.minus(new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(41.375)/4, 0));
    public static final Translation3d kShooterRightCenter = kShooterCenter.plus(new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(41.375)/4, 0));
    public static final double kShooterWidth = Units.inchesToMeters(41.375);
    public static final double kShooterHeight = Units.inchesToMeters(4.875 * 2);
    public static final double kShooterDepth = Units.inchesToMeters(18);
    public static final Translation3d kShooterBackLeft = new Translation3d(Units.inchesToMeters(0), kShooterBaseLength+kShooterWidth/2,kShooterBaseHeight); 
    public static final Translation3d kShooterBackRight = new Translation3d(Units.inchesToMeters(0), kShooterBaseLength-kShooterWidth/2, kShooterBaseHeight);
    public static final Translation3d kShooterFrontLeft = new Translation3d(kShooterDepth, kShooterBaseLength+kShooterWidth/2, kShooterBaseHeight + kShooterHeight/2);
    public static final Translation3d kShooterFrontRight = new Translation3d(kShooterDepth, kShooterBaseLength-kShooterWidth/2, kShooterBaseHeight+ kShooterHeight/2);
    
    // public static final Pose3d kShooterMiddle = Pose3d()
    
  }

  public static final class ClimbConstants {
    public static final int kServoPort = 9;
    public static final int kServoPort2 = 8;
    public static final double kLockPosition = 0.2;
    public static final double kUnlockPosition = 0.0;

    public static final double gearboxRatio = 10.5;
  }

  public static final class IndexerConstants {
    public static final double kIndexerSpeed = 50;
    public static final double kIndexerLength = Units.inchesToMeters(15);
    public static final double kRollerDiameter = Units.inchesToMeters(1.6);    
    public static final double gearboxRatio = 1;
    


    public static final LoggedTunableNumber kIndexerP = new LoggedTunableNumber("Indexer P", 1, "Indexer");
    public static final LoggedTunableNumber kIndexerI = new LoggedTunableNumber("Indexer I", 0.0, "Indexer");
    public static final LoggedTunableNumber kIndexerD = new LoggedTunableNumber("Indexer D", 0.05, "Indexer");
  
    public static final LoggedTunableNumber kShootingSpeed = new LoggedTunableNumber("Shooting Speed", 0.5, "Indexer");

  }

  public static final class ShooterMathConstants {
    public static final double cubicA = 0.0262588;
    public static final double cubicB = -0.894156;
    public static final double cubicC = 10.3972;
    public static final double cubicD = 1.85749;


    
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
  }

  public static final class TrajectoryGenerationManager {
    public static final String kTrajectoryTableName = "Trajectories";
  }
}
