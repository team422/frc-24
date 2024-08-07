package frc.robot;
import java.util.HashMap;

import org.apache.commons.math3.analysis.function.Log;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;

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
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.utils.Alert;
import frc.lib.utils.Alert.AlertType;
import frc.lib.utils.CustomHolmonomicDrive;
import frc.lib.utils.LoggedTunableNumber;
import frc.lib.utils.TunableNumber;
import frc.robot.utils.Note;
import frc.robot.utils.swerve.ModuleLimits;


public final class Constants {
    public static final boolean tuningMode = true;
    public static final boolean isTunableNetwork = false;
    private static RobotType robotType = RobotType.COMPBOT;
    public static final Mode curRobotMode = Mode.REPLAY;
    public static RobotType getRobot() {
    if (RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR)
          .set(true);
      robotType = RobotType.COMPBOT;
    }
    return robotType;
  }
    public static Mode getMode() {
      return switch (robotType) {
        case  COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
        case SIMBOT -> Mode.SIM;
      };
    }
    public enum RobotType {
      SIMBOT,
      COMPBOT
    }
    public enum Mode {
      /** Running on a real robot. */
      REAL,
  
      /** Running a physics simulator. */
      SIM,
  
      /** Replaying from a log file. */
      REPLAY
    }
    // public static final TunableNumber tuning = new TunableNumber("Tuning Enabled", 0,"Tuning");
    public static final boolean fullManualShooterAndPivotSpeedControls = false;
    public static final boolean fullManualIntakePivotAndSpeedControls = false;
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
    public static final int beamBreakPort = 8;
    public static final int beamBreakPort2 = 7;

    public static final int shooterLeft = 35;
    public static final int shooterRight = 36;

    public static final int shooterPivot = 39;
    public static final int shooterPivotFollower = 40;

    public static final int ledPort = 9;

    public static final int ampMotor = 54;
  }



  public static final class RobotConstants {
    public static final double trackWidth = Units.inchesToMeters(23.5);
    public static final boolean AScopeLogging = true;
  }

  public static final class IntakeConstants {
    public static final double intakeSpeed = 0.75;
    public static final double intakeSpeedToMPS = 1 * Units.inchesToMeters(1.5) * Math.PI;
    public static final double pivotGearRatio = 36.0/16;
    public static final Rotation2d kIntakeMaxMovedAngle = Rotation2d.fromDegrees(90);
    public static final Rotation2d kIntakeMaxAngle = Rotation2d.fromDegrees(118);
    public static final Rotation2d kIntakeMinAngle = Rotation2d.fromDegrees(9.3);
    public static final Rotation2d kIntakeHomeAngle = Rotation2d.fromDegrees(29);
    public static final Rotation2d kAmpAngle = Rotation2d.fromDegrees(70);

    // public static final double kIntakePivotManualControl = 0;
    public static final LoggedTunableNumber kIntakePivotManualControl = new LoggedTunableNumber("Intake Pivot Manual Control", 0.5, "Intake Manual");
    public static final LoggedTunableNumber kIntakeRollerManualControl = new LoggedTunableNumber("Intake Roller Manual Control", 0.5, "Intake Manual");

    public static final double kFlywheelTolerance = 15;

    public static final double kIntakeStowTime = 0.75; // Seconds from indexer receiving game piece to stowing the intake

    public static final LoggedTunableNumber kIntakeP = new LoggedTunableNumber("Intake P", .6, "Intake");
    public static final LoggedTunableNumber kIntakeI = new LoggedTunableNumber("Intake I", 0.0, "Intake");
    public static final LoggedTunableNumber kIntakeD = new LoggedTunableNumber("Intake D", 0.05, "Intake");
    public static final LoggedTunableNumber kIntakeKS = new LoggedTunableNumber("Intake KS", 0.0,"Intake FF");
    public static final LoggedTunableNumber kIntakeKA = new LoggedTunableNumber("Intake KA", 0.0,"Intake FF");
    public static final LoggedTunableNumber kIntakeKG = new LoggedTunableNumber("Intake KG", 0.0,"Intake FF");


  }

  public static final class ShooterConstants {

    public static final class FlywheelConstants {
      public static final double kFlywheelGearRatio = 1/1.5;
      public static final double kFlywheelDiameter = Units.inchesToMeters(4);
      public static final LoggedTunableNumber kFlywheelP = new LoggedTunableNumber("Flywheel P", -0.035000, "Shooter");
      public static final LoggedTunableNumber kFlywheelI = new LoggedTunableNumber("Flywheel I", 0.0, "Shooter");
      public static final LoggedTunableNumber kFlywheelISIM = new LoggedTunableNumber("Flywheel I SIM", 0.0, "Shooter");
      public static final LoggedTunableNumber kFlywheelD = new LoggedTunableNumber("Flywheel D", 0.0003, "Shooter");
      public static final LoggedTunableNumber kFlywheelKSLeft = new LoggedTunableNumber("Flywheel KS Left", 0.270000, "Shooter");
      public static final LoggedTunableNumber kFlywheelKVLeft = new LoggedTunableNumber("Flywheel KV Left", 0.125000, "Shooter");
      public static final LoggedTunableNumber kFlywheelKALeft = new LoggedTunableNumber("Flywheel KA Left", 0.00, "Shooter");
      public static final LoggedTunableNumber kFlywheelKSRight = new LoggedTunableNumber("Flywheel KS Right", 0.410000, "Shooter");
      public static final LoggedTunableNumber kFlywheelKVRight = new LoggedTunableNumber("Flywheel KV Right", 0.126000, "Shooter");
      public static final LoggedTunableNumber kFlywheelKARight = new LoggedTunableNumber("Flywheel KA Right", 0.00, "Shooter");
      public static final LoggedTunableNumber kFlywheelAccel = new LoggedTunableNumber("Flywheel Accel", 0, "Shooter");

      public static final double kMaxSpeed = 100;
      public static final double kIdleSpeed = 14;
      public static final double kIdleSpeedClose = 12;
      public static final double kIdleSpeedFar = 21;
      public static final LoggedTunableNumber kFlywheelSpeedLeft = new LoggedTunableNumber("Flywheel Speed Left", 0.0, "Shooter");
      public static final LoggedTunableNumber kFlywheelSpeedRight = new LoggedTunableNumber("Flywheel Speed Right", 0.0, "Shooter");
      public static final LoggedTunableNumber kAmpSpeed = new LoggedTunableNumber("Flywheel amp speed", 5.70000, "Amp");
      public static final LoggedTunableNumber kFlywheelHockeyPuck = new LoggedTunableNumber("Flywheel hockey puck speed", 9.5, "Hockey Puck");
      public static final LoggedTunableNumber kSourceIntakeSpeed =  new LoggedTunableNumber("Source intake speed", -12, "Source Intake");
      public static final LoggedTunableNumber kSourceIntakeStopTime = new LoggedTunableNumber("Source intake Stop time", .4, "Source Intake");
    }

    public static final class ShooterPivotConstants {
      // Real constants
      public static final double gearboxRatio = 36.2500;
      public static final LoggedTunableNumber kPivotP = new LoggedTunableNumber("Pivot P", 130.0, "Shooter");

      public static final LoggedTunableNumber kPivotI = new LoggedTunableNumber("Pivot I", 0.0, "Shooter");
      public static final LoggedTunableNumber kPivotD = new LoggedTunableNumber("Pivot D", 8.000000, "Shooter");
      public static final LoggedTunableNumber kPivotAmpP = new LoggedTunableNumber("Pivot Amp P ", 1.0, "Shooter");
      public static final LoggedTunableNumber kPivotAmpI = new LoggedTunableNumber("Pivot Amp I ", 0.0, "Shooter");
      public static final LoggedTunableNumber kPivotAmpD = new LoggedTunableNumber("Pivot Amp D ", 0.0, "Shooter");
      public static final LoggedTunableNumber kPivotkS = new LoggedTunableNumber("Pivot KS", 0.200000, "Shooter");

      public static final LoggedTunableNumber kPivotkV = new LoggedTunableNumber("Pivot KV", 0.0, "Shooter");
      public static final LoggedTunableNumber kPivotkA = new LoggedTunableNumber("Pivot KA", 0.0, "Shooter");
      public static final LoggedTunableNumber kPivotkG = new LoggedTunableNumber("Pivot KG", 0.080000, "Shooter"); 
      public static final LoggedTunableNumber kUsingAmp = new LoggedTunableNumber("Pivot using amp", 0, "Shooter"); 

      public static final LoggedTunableNumber kPivotPDirect = new LoggedTunableNumber("Pivot P Direct", 6.0, "Shooter");
      public static final LoggedTunableNumber kPivotIDirect = new LoggedTunableNumber("Pivot I Direct", 3.000000, "Shooter");
      public static final LoggedTunableNumber kPivotDDirect = new LoggedTunableNumber("Pivot D Direct", 0.0, "Shooter");
      public static final LoggedTunableNumber kSourceIntake = new LoggedTunableNumber("Source Intake angle", 56.0, "Shooter");

      public static final double kOffset = Rotation2d.fromDegrees(181.7-13+52+6+123).getRotations();

      public static final LoggedTunableNumber kHockeyPuck = new LoggedTunableNumber("hockey puck", 53.0,"Hockey Puck");

      public static final LoggedTunableNumber kShooterAngle = new LoggedTunableNumber("Shooter Angle", 0.0, "Shooter");
      // public static final double kOffset = Rotation2d.fromDegrees(0).getRotations();

      
      public static final Rotation2d maxAngle = Rotation2d.fromDegrees(77); 
      public static final LoggedTunableNumber kAmpShot = new LoggedTunableNumber("Shooter Amp Angle",50.000000, "Amp");
      public static final Rotation2d kAmpBottom = Rotation2d.fromDegrees(50);
      public static final Rotation2d minAngle = Rotation2d.fromDegrees(15);
      public static final Rotation2d homeAngle = Rotation2d.fromDegrees(36);
      public static final Rotation2d kFenderAngle = Rotation2d.fromDegrees(57);// 57
      public static final LoggedTunableNumber kCloseThreeAngle = new LoggedTunableNumber("CloseThreeAngle", 33,"Shooter"); 


      public static final double maxSpeed = 1.0;
      public static final double maxAcceleration = 1.0;

      public static final Translation3d topRightSpeaker =
        new Translation3d(
            Units.inchesToMeters(18.055),
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

      // SIM
      public static final Rotation2d simOffset = Rotation2d.fromDegrees(35);

    }
  }


  public static final class AmpConstants {
    public static final LoggedTunableNumber kAmpShot = new LoggedTunableNumber("Amp Shot", 196.0, "Amp");
    public static final LoggedTunableNumber kAmpAngle = new LoggedTunableNumber("Amp Angle",196.0, "Amp");
    public static final LoggedTunableNumber kAmpP = new LoggedTunableNumber("Amp P", 6.0, "Amp");
    public static final LoggedTunableNumber kAmpI = new LoggedTunableNumber("Amp I", 0.0, "Amp");
    public static final LoggedTunableNumber kAmpD = new LoggedTunableNumber("Amp D", 0.0, "Amp");
  }

  public static final class DriveConstants {
    public static final Integer kId = 1;
    public static final Integer kOdometryFrequency = 250;
    public static final double kMaxSpeedMetersPerSecond = 5.8;
    public static final double kMaxAngularSpeedRadiansPerSecond = 1.75 * Math.PI;
    public static final LoggedTunableNumber driveAccel = new LoggedTunableNumber("drive accel",.3,"Drive");
    public static final double controllerDeadzone = 0.04;
    public static final double kShootToleranceDeg = 15;
    public static final double kWheelRadius = Units.inchesToMeters(13.2582521472);
    public static final double kWheelBase = Units.inchesToMeters(18.75);
    public static final double kTrackWidth = Units.inchesToMeters(18.75);
    
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
    public static final double kMaxAngula1AccelRadiansPerSecondSq = 3.0 * Math.PI; 

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
    public static final PathConstraints kAutoAlignToAmpSpeed = new PathConstraints(1.0,2.0,2,2);
    public static final PathConstraints kDriveToPieceSpeed = new PathConstraints(3.5,1.0,1,1);
    public static final LoggedTunableNumber AutoAlignP = new LoggedTunableNumber("Auto Align P", 6., "Drive");
    public static final LoggedTunableNumber AutoAlignD = new LoggedTunableNumber("Auto Align D", 0.0, "Drive");
    public static final LoggedTunableNumber kHeadingP = new LoggedTunableNumber("kHeadingP", 1.000 , "Drive");
    public static final LoggedTunableNumber kHeadingI = new LoggedTunableNumber("kHeadingI", 0.000 , "Drive");
    public static final LoggedTunableNumber kHeadingD = new LoggedTunableNumber("kHeadingD", 0.030000 , "Drive");
    public static final LoggedTunableNumber kHolonomicXYP = new LoggedTunableNumber("kHolnomicXYP",0.24,"Drive");
    public static final LoggedTunableNumber kHolonomicXYI = new LoggedTunableNumber("kHolnomicXYI",.0,"Drive");
    public static final LoggedTunableNumber kHolonomicXYD = new LoggedTunableNumber("kHolnomicXYD",.0,"Drive");
    public static final LoggedTunableNumber kHolonomicThetaP = new LoggedTunableNumber("kHolnomicThetaP",.45,"Drive");
    public static final LoggedTunableNumber kHolonomicThetaI = new LoggedTunableNumber("kHolnomicThetaI",.0,"Drive");
    public static final LoggedTunableNumber kHolonomicThetaD = new LoggedTunableNumber("kHolnomicThetaD",.0,"Drive");
    
  }

  public static final class ModuleConstants {

    public static final LoggedTunableNumber currentAngleToGoTo = new LoggedTunableNumber("Angle to set wheels to", 0,"drive");
    public static final LoggedTunableNumber turnStatic = new LoggedTunableNumber("drive ff", -0.200000,"drive");


    public static final LoggedTunableNumber kDriveP = new LoggedTunableNumber("Drive wP", 0.150000, "Drive");
    public static final LoggedTunableNumber kDriveI = new LoggedTunableNumber("Drive I", 0.0, "Drive");
    public static final LoggedTunableNumber kDriveD = new LoggedTunableNumber("Drive D", 0.00, "Drive");
    public static final LoggedTunableNumber kDriveFF = new LoggedTunableNumber("Drive FF", 2.96, "Drive");

    public static final LoggedTunableNumber kffkS = new LoggedTunableNumber("FF KS", 0.274890, "Drive");
    public static final LoggedTunableNumber kffkV = new LoggedTunableNumber("FF KV", 2.083780, "Drive");
    public static final LoggedTunableNumber kffkA = new LoggedTunableNumber("FF KA", 3.583780, "Drive");

    public static final LoggedTunableNumber kFFDriveP = new LoggedTunableNumber("FFDrive wP", 3., "Drive");
    public static final LoggedTunableNumber kFFDriveI = new LoggedTunableNumber("FFDrive I", 0.0, "Drive");
    public static final LoggedTunableNumber kFFDriveD = new LoggedTunableNumber("FFDrive D", 0.00, "Drive");

    public static final LoggedTunableNumber FLkTurningP = new LoggedTunableNumber("FL Turning P", 6.0, "Drive");
    public static final LoggedTunableNumber FLkTurningI = new LoggedTunableNumber("FL Turning I", 3.00, "Drive");
    public static final LoggedTunableNumber FLkTurningD = new LoggedTunableNumber("FL Turning D", 0.0003 , "Drive");
    public static final LoggedTunableNumber FRkTurningP = new LoggedTunableNumber("FR Turning P", 6.0, "Drive");
    public static final LoggedTunableNumber FRkTurningI = new LoggedTunableNumber("FR Turning I", 3.00, "Drive");
    public static final LoggedTunableNumber FRkTurningD = new LoggedTunableNumber("FR Turning D", 0.0003 , "Drive");
    public static final LoggedTunableNumber BLkTurningP = new LoggedTunableNumber("BL Turning P", 6.0, "Drive");
    public static final LoggedTunableNumber BLkTurningI = new LoggedTunableNumber("BL Turning I", 3.00, "Drive");
    public static final LoggedTunableNumber BLkTurningD = new LoggedTunableNumber("BL Turning D", 0.0003 , "Drive");
    public static final LoggedTunableNumber BRkTurningP = new LoggedTunableNumber("BR Turning P", 6.0, "Drive");
    public static final LoggedTunableNumber BRkTurningI = new LoggedTunableNumber("BR Turning I", 3.00, "Drive");
    public static final LoggedTunableNumber BRkTurningD = new LoggedTunableNumber("BR Turning D", 0.0003 , "Drive");
     

    



    public static final LoggedTunableNumber kDriveKS = new LoggedTunableNumber("Drive KS", 0.28637, "Drive");
    public static final LoggedTunableNumber kDriveKV = new LoggedTunableNumber("Drive KV", .12202, "Drive");
    public static final LoggedTunableNumber kDriveKA = new LoggedTunableNumber("Drive KA", 0, "Drive");

    public static final LoggedTunableNumber kTurningKS = new LoggedTunableNumber("Turning KS", 0.3, "Drive");
    public static final LoggedTunableNumber kTurningKV = new LoggedTunableNumber("Turning KV", 1.5, "Drive");
    public static final LoggedTunableNumber kTurningKA = new LoggedTunableNumber("Turning KA", 0, "Drive");


    public static final LoggedTunableNumber kTurningKP = new LoggedTunableNumber("Turning KP", 100., "Drive");
    public static final LoggedTunableNumber kTurningKI = new LoggedTunableNumber("Turning KI", 0.0, "Drive");
    public static final LoggedTunableNumber kTurningKD = new LoggedTunableNumber("Turning KD", 0, "Drive");


    public static final LoggedTunableNumber kTurningPSim = new LoggedTunableNumber("TurningP Sim", 4.5, "Drive");
    public static final LoggedTunableNumber kTurningISim = new LoggedTunableNumber("Turning I Sim", 0.00, "Drive");
    public static final LoggedTunableNumber kTurningDSim = new LoggedTunableNumber("Turning D Sim", 0.005, "Drive");

    public static final LoggedTunableNumber kStartAccel = new LoggedTunableNumber("Start Accel", 4.5, "Drive");
    public static final LoggedTunableNumber kAccelDropoff = new LoggedTunableNumber("Accel Dropoff", -1, "Drive");

    public static final LoggedTunableNumber kStartAccelSim = new LoggedTunableNumber("Start Accel Sim", 4.5, "Drive");
    public static final LoggedTunableNumber kAccelDropoffSim = new LoggedTunableNumber("Accel Dropoff Sim", -1, "Drive");

    public static final double kWheelDiameterMeters = Units.inchesToMeters(1.945*2); // 0.09398; // 3.7 in

    public static final double kDriveGearRatio = 6.12244897959; // OLD 6.75
    public static final double kDriveConversionFactor = ((kDriveGearRatio/ ((Units.inchesToMeters(4) * Math.PI) )));
    public static final double kDriveConversionFactorSim = 1/(Units.inchesToMeters(4)*Math.PI);
    // public static final double kDriveConversionFactor = 1 / 22.0409;

    // public static final ModuleLimits freeSpeedLimits = new ModuleLimits(5.9, Units.feetToMeters(75), Rotation2d.fromDegrees(1080).getRadians());

    

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
    public static final Pose2d kRedAllianceOrigin = new Pose2d(kFieldLengthMeters, kFieldWidthMeters,
        Rotation2d.fromDegrees(180));

    public static final double kShooterBaseHeight = Units.inchesToMeters(79.129 + 1/8);// ONE INCH HAS BEEN ADDED BECAUSE VISUAL MODELS SHOW IT TO BE ONE INCH HIGHER, CHECK IN REAL LIFE @JAMES REMINDER
    public static final double kShooterTopHeight = Units.inchesToMeters(98.25);
    public static final double kShooterBaseLength = Units.inchesToMeters(217.9585);
    public static final Rotation2d kShooterAngle = Rotation2d.fromDegrees(14);
    public static final Translation3d kShooterCenter = new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(217.9585), Units.inchesToMeters(84));
    public static final Translation2d kshooterBase = new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(217.9585));
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
    public static final double kIndexerShootingSpeed = 10000;
    public static final double kIndexerSpeedAuto = 100;
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

    public static final LoggedTunableNumber allowedDistance = new LoggedTunableNumber("Allowed Vision distance", 5, "Shooter Math") ;

    // error cubic constants
    public static final double errorCubicA = -0.00000216579;
    public static final double errorCubicB = 0.00139213;
    public static final double errorCubicC = -0.296805;
    public static final double errorCubicD = 23.9083;
    public static final LoggedTunableNumber pivotErrorMultiplier = new LoggedTunableNumber("Pivot Error Multiplier", 15, "Shooter Math");
    public static final LoggedTunableNumber FlywheelDropoffInitilizer = new LoggedTunableNumber("Flywheel dropoff init", 7, "Shooter Math");
    public static final LoggedTunableNumber FlywheelDropoffMultiplier = new LoggedTunableNumber("Flywheel multiplier", -.05, "Shooter Math");
    public static final LoggedTunableNumber FlywheelDropoffFloor = new LoggedTunableNumber("Flywheel Floor", 1.500000, "Shooter Math");
    public static final LoggedTunableNumber headingErrorMultiplier = new LoggedTunableNumber("headingErrorMultiplier", 2.8, "Shooter Math");
    public static final LoggedTunableNumber PivotDropoffInitilizer = new LoggedTunableNumber("Pivot dropoff init", 7, "Shooter Math");
    public static final LoggedTunableNumber PivotDropoffMultiplier = new LoggedTunableNumber("Pivot multiplier", -.05, "Shooter Math");
    public static final LoggedTunableNumber PivotDropoffFloor = new LoggedTunableNumber("Pivot Floor", .500000, "Shooter Math");


    
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
      public static final LoggedTunableNumber distanceClose = new LoggedTunableNumber("distance close width", 0.56,"Objectmagic");
      public static final LoggedTunableNumber distanceFar = new LoggedTunableNumber("distance far width", 1.5,"Objectmagic");
      public static final LoggedTunableNumber angleDepression = new LoggedTunableNumber("angle Max", 78,"Objectmagic");
      public static final LoggedTunableNumber angleMin = new LoggedTunableNumber("angle min", 52.25,"Objectmagic");

      
    }
  }

  public static final class AutoRuns {

    public static final HashMap<String,Note[]> scoutPossibilties = new HashMap<String,Note[]>()
    {{
      put("456", new Note[]{Note.FOUR, Note.FIVE, Note.SIX});
      put("678", new Note[]{Note.SIX, Note.SEVEN, Note.EIGHT});
      put("567", new Note[]{Note.FIVE, Note.SIX, Note.SEVEN});

      
    }};
    public static final HashMap<String, Integer> scoutNumberPossibilities = new HashMap<String, Integer>() {{
      put("1", 1);
      put("2", 2);
      put("3", 3);
      put("4", 4);
  }};
    public static final HashMap<String,Note[]> endPossibilties = new HashMap<String,Note[]>()
      {{
        put("1", new Note[]{Note.ONE});
        put("2", new Note[]{Note.TWO});
        put("3", new Note[]{Note.THREE});
        
      
      }}
    ;

    public static final Pose2d farSourceWingShot = new Pose2d(4.67,6.16,Rotation2d.fromDegrees(1.24));
    public static final Pose2d underStageShot = new Pose2d(4.20,4.94,Rotation2d.fromDegrees(-12.03));
    public static final Pose2d ampSideWingShot = new Pose2d(3.87,2.76,Rotation2d.fromDegrees(139.53));


    

    // queue possibilties


  }

  public static final class TrajectoryGenerationManager {
    public static final String kTrajectoryTableName = "Trajectories";
  }
}
