package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.hardwareprofiler.ProfilingScheduling;
import frc.lib.utils.SwerveTester;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.FlywheelConstants;
import frc.robot.Constants.ShooterConstants.ShooterPivotConstants;
import frc.robot.commands.autonomous.CustomTrajectoryRunner;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveProfiles;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.northstarAprilTagVision.AprilTagVision;
import frc.robot.subsystems.objectVision.ObjectDetectionCam;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.ShooterMath;
import frc.robot.subsystems.climb.Climb;


public class RobotState {
    // singleton class that handles all inter subsystem communication
    private static RobotState instance = null;
    private Drive m_drive;
    private Climb m_climb;
    private Indexer m_indexer;
    private Shooter m_shooter;
    private ObjectDetectionCam m_objectDetectionCams;
    private AprilTagVision m_aprilTagVision;
    private Intake m_intake;
    private ShooterMath m_shooterMath;
    private SwerveTester m_swerveTester;
    private CustomTrajectoryRunner m_customTrajectoryRunner;

    public enum RobotCurrentAction {
      kStow,kIntake,kShootFender,kRevAndAlign, kShootWithAutoAlign,kAmpLineup,kAmpShoot
    }

    public RobotCurrentAction curAction = RobotCurrentAction.kStow;

    public Rotation2d shooterOverrideAngle = null;
    public long angleOverrideTime = 0;

    public double intakeStowTime = -1;
    public double shooterStopTime = -1;

    public boolean kAmpTopReached = false;

  public record OdometryObservation(
      SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, double timestamp) {}

    public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}

    public enum GamePieceLocation {
        INTAKE,
        INDEXER,
        SHOOTER,
        NOT_IN_ROBOT
    }

    private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
  private SwerveDriveWheelPositions lastWheelPositions =
      new SwerveDriveWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
          });
  private Rotation2d lastGyroAngle = new Rotation2d();
  private Twist2d robotVelocity = new Twist2d();
    private GamePieceLocation m_gamePieceLocation = GamePieceLocation.NOT_IN_ROBOT;
    public List<Pose2d> activePath = new ArrayList<Pose2d>();
    private static final double poseBufferSizeSeconds = 2.0;

private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);
      private Pose2d odometryPose = new Pose2d();
        private Pose2d estimatedPose = new Pose2d();
    public Pose2d currentTarget = new Pose2d();
    public LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<Command>("Auto Chooser");

    private RobotState(Drive drive, Climb climb, Indexer indexer, Shooter shooter, ObjectDetectionCam objectDetectionCams, AprilTagVision aprilTagVisions, Intake intake) {
        this.m_drive = drive;
        this.m_climb = climb;
        this.m_shooter = shooter;
        this.m_indexer = indexer;
        this.m_objectDetectionCams = objectDetectionCams;
        this.m_aprilTagVision = aprilTagVisions;
        this.m_intake = intake;
        m_shooterMath = new ShooterMath(new Translation3d(0,0,Units.inchesToMeters(16.496)));
        ProfilingScheduling.startInstance();
        for (int i = 0; i < 3; ++i) {
      qStdDevs.set(i, 0, Math.pow(DriveConstants.odometryStateStdDevs.get(i, 0), 2));
    }
        
        m_customTrajectoryRunner = new CustomTrajectoryRunner();
        // this.m_swerveTester = new SwerveTester(drive);
    }
    public static RobotState startInstance(Drive drive, Climb climb, Indexer indexer, Shooter shooter, ObjectDetectionCam objectDetectionCams, AprilTagVision aprilTagVision, Intake intake) {
        if (instance == null) {
            
            instance = new RobotState(drive, climb, indexer,shooter, objectDetectionCams, aprilTagVision, intake);

        }
        return instance;
    }



   
    public static RobotState getInstance() {
        
        return instance;
    }

    public void addVelocityData(Twist2d robotVelocity) {
      this.robotVelocity = robotVelocity;
    }
  

      public void addVisionObservation(VisionObservation observation) {
    // latestParameters = null;
    // If measurement is old enough to be outside the pose buffer's timespan, skip.
    try {
      if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSeconds
          > observation.timestamp()) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }
    // Get odometry based pose at timestamp
    var sample = poseBuffer.getSample(observation.timestamp());
    if (sample.isEmpty()) {
      // exit if not there
      System.out.println("Sample is empty");
      return;
    }

    
    

    // sample --> odometryPose transform and backwards of that
    var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
    var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
    // get old estimate by applying odometryToSample Transform
    Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

    // Calculate 3 x 3 vision matrix
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
    }
    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      double stdDev = qStdDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
      }
    }
    // difference between estimate and vision pose
    Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose());
    // scale transform by visionK
    var kTimesTransform =
        visionK.times(
            VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    Transform2d scaledTransform =
        new Transform2d(
            kTimesTransform.get(0, 0),
            kTimesTransform.get(1, 0),
            Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

    // Recalculate current estimate by applying scaled transform to old estimate
    // then replaying odometry data
    estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
  }

  public void goToShootPositionAndRev(){

    
    
  }

  public void setRobotCurrentAction(RobotCurrentAction action){
    if(curAction == RobotCurrentAction.kAmpLineup){
      ShooterPivotConstants.kUsingAmp.set(0);
      
    }
    if(action == RobotCurrentAction.kAmpLineup){
      ShooterPivotConstants.kUsingAmp.set(1);
      kAmpTopReached = false;
    }
    curAction = action;
  }

  /** Add odometry observation */
  public void addOdometryObservation(OdometryObservation observation) {

    Twist2d twist = DriveConstants.kDriveKinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
    lastWheelPositions = observation.wheelPositions();
    // Check gyro connected
    if (observation.gyroAngle != null) {
      // Update dtheta for twist if gyro connected
      twist =
          new Twist2d(
              twist.dx, twist.dy, observation.gyroAngle().minus(lastGyroAngle).getRadians());
      lastGyroAngle = observation.gyroAngle();
    }
    // Add twist to odometry pose
    odometryPose = odometryPose.exp(twist);
    // Add pose to buffer at timestamp
    poseBuffer.addSample(observation.timestamp(), odometryPose);
    // Calculate diff from last odometry pose and add onto pose estimate
    estimatedPose = estimatedPose.exp(twist);
  }

    public void saveActivePath(List<Pose2d> path) {
        activePath = path;
        if(path.size() > 0){
        Logger.recordOutput("next target", path.get(path.size()-1));
        }

    }

    public void resetPose(Pose2d initialPose) {
        estimatedPose = initialPose;
        odometryPose = initialPose;
        poseBuffer.clear();
      }

    public void saveCurrentPose(Pose2d pose) {
        }

    public void saveCurrentTarget(Pose2d pose) {
        currentTarget = pose;
        Logger.recordOutput("target pose",pose);

        }

    public Rotation2d getMaxIntakeAngle() {
        // first get desired angle of the shooter
        Rotation2d shooterAngle = m_shooter.getPivotAngle();
        if (shooterAngle.getDegrees() < 25) {
            return IntakeConstants.kIntakeMaxMovedAngle;
        }
        return IntakeConstants.kIntakeMaxAngle;
    }


    public void setGamePieceLocation(GamePieceLocation location) {
        m_gamePieceLocation = location;
        if (location == GamePieceLocation.INDEXER) {
            m_indexer.setState(Indexer.IndexerState.INDEXING);
            m_shooter.isIntaking = Shooter.ShooterIsIntaking.notIntaking;
            curAction = RobotCurrentAction.kStow;
            stowShooter();
            stowAndStopIntake();
        }
        if (location == GamePieceLocation.SHOOTER){
            shooterStopTime = Timer.getFPGATimestamp() + 1;
            Logger.recordOutput("knows it shot", shooterStopTime);

        }
    }

    public void stowShooter() {
        m_shooter.setPivotAngle(Rotation2d.fromDegrees(31));
    }

    public void stowAndStopIntake() {
        m_intake.setIntakeSpeed(0);
        m_intake.setPivotAngle(IntakeConstants.kIntakeMaxAngle);
    }


    public Pose2d getRobotPose() {
        return m_drive.getPose();
    }

    public Optional<Rotation2d> getOptionalTargetOverride() {
        if (shooterOverrideAngle != null && angleOverrideTime > System.currentTimeMillis()) {
            return Optional.of(shooterOverrideAngle);
        } 
        return Optional.empty();
    }

    public Pose2d getDesiredRobotPose(){
        // return m_drive.getDesiredPose();
        return new Pose2d();
    }

    public void goToIntakePosition() {
        m_intake.setPivotAngle(IntakeConstants.kIntakeMinAngle);
        m_shooter.setPivotAngle(ShooterPivotConstants.homeAngle);
    }

    public void updateTestScheduler(){
        if (ProfilingScheduling.hasTests() && !DriverStation.isFMSAttached()) {
            ProfilingScheduling.updateCurrent();
        }
    }

    public void updateRobotState() {
        if(intakeStowTime != -1 && intakeStowTime < Timer.getFPGATimestamp()) {
            intakeStowTime = -1;
        }
        if (shooterStopTime != -1 && shooterStopTime < Timer.getFPGATimestamp()) {
            m_shooter.setFlywheelSpeed(ShooterConstants.FlywheelConstants.kIdleSpeed);
            if(curAction != RobotCurrentAction.kAmpLineup){
              curAction = RobotCurrentAction.kStow;
            }
            setGamePieceLocation(GamePieceLocation.NOT_IN_ROBOT);
            m_indexer.setState(IndexerState.IDLE);
            stowShooter();
            shooterStopTime = -1;
            Logger.recordOutput("knows it shot", shooterStopTime);
        }

        if (curAction == RobotCurrentAction.kStow) {
          m_drive.setProfile(DriveProfiles.kDefault);
          stowAndStopIntake();
          stowShooter();
          m_shooter.setFlywheelSpeed(0);
          // m_indexer.setState(IndexerState.IDLE);
        } else if (curAction == RobotCurrentAction.kIntake){
          m_drive.setProfile(DriveProfiles.kDefault);
          
          m_indexer.setState(IndexerState.INTAKING);

          m_intake.setIntakeSpeed(IntakeConstants.intakeSpeed);
          goToIntakePosition();
          m_shooter.setFlywheelSpeed(0);
        }else if (curAction == RobotCurrentAction.kShootFender){
          m_shooter.setPivotAngle(ShooterPivotConstants.kFenderAngle);
          // m_indexer.setState(Indexer.IndexerState.SHOOTING);
          if (m_shooter.isWithinTolerance(((m_shooterMath.getShooterMetersPerSecond(2).get(0)+m_shooterMath.getShooterMetersPerSecond(2).get(1)))/2.0) && m_shooter.isPivotWithinTolerance(ShooterPivotConstants.kFenderAngle,Rotation2d.fromDegrees(5))) {
            m_indexer.setState(Indexer.IndexerState.SHOOTING);

          } 
          

        } else if (curAction == RobotCurrentAction.kRevAndAlign){
          Pose2d predPose = getPredictedPose(0.0,0.0);

          ArrayList<Rotation2d> mRotations = m_shooterMath.setNextShootingPoseAndVelocity(predPose,robotVelocity,FieldConstants.kShooterCenter);
          // double speed = m_shooterMath.getShooterMetersPerSecond(m_shooterMath.getDistanceFromTarget(predPose,FieldConstants.kShooterCenter));
          
          m_drive.setProfile(DriveProfiles.kAutoAlign);
          m_shooter.setFlywheelSpeedWithSpin(m_shooterMath.getShooterMetersPerSecond(m_shooterMath.getDistanceFromTarget(predPose,FieldConstants.kShooterCenter)).get(0),m_shooterMath.getShooterMetersPerSecond(m_shooterMath.getDistanceFromTarget(predPose,FieldConstants.kShooterCenter)).get(1));
          m_shooter.setPivotAngle(mRotations.get(1));
          m_drive.setDriveTurnOverride(mRotations.get(0));

        } else if (curAction == RobotCurrentAction.kShootWithAutoAlign){
          Pose2d predPose = getPredictedPose(0.4,0.4);
          ArrayList<Rotation2d> mRotations = m_shooterMath.setNextShootingPoseAndVelocity(predPose,robotVelocity,FieldConstants.kShooterCenter);
          // double speed = m_shooterMath.getShooterMetersPerSecond(m_shooterMath.getDistanceFromTarget(predPose,FieldConstants.kShooterCenter));
          m_drive.setProfile(DriveProfiles.kAutoAlign);
          // m_shooter.setFlywheelSpeed(speed);
          m_shooter.setFlywheelSpeedWithSpin(m_shooterMath.getShooterMetersPerSecond(m_shooterMath.getDistanceFromTarget(predPose,FieldConstants.kShooterCenter)).get(0),m_shooterMath.getShooterMetersPerSecond(m_shooterMath.getDistanceFromTarget(predPose,FieldConstants.kShooterCenter)).get(1));
          m_shooter.setPivotAngle(mRotations.get(1));
          m_drive.setDriveTurnOverride(mRotations.get(0));
          if (Math.abs(getEstimatedPose().getRotation().minus(mRotations.get(0)).getDegrees()) < DriveConstants.kShootToleranceDeg) {
            m_indexer.setState(Indexer.IndexerState.SHOOTING);

          }

        } else if (curAction == RobotCurrentAction.kAmpLineup){
          // m_drive.setProfile(DriveProfiles.kTrajectoryFollowing);
          Logger.recordOutput("Starting amp",kAmpTopReached);
          m_shooter.setFlywheelSpeedWithSpin(FlywheelConstants.kAmpSpeed.get(),FlywheelConstants.kAmpSpeed.get());
          if(kAmpTopReached==false){
            m_shooter.setPivotAngle(ShooterPivotConstants.maxAngle);
          }
          

          if(kAmpTopReached == true){
            m_indexer.setState(IndexerState.SHOOTING);

            if(shooterStopTime != -1 && shooterStopTime < Timer.getFPGATimestamp()){

              m_shooter.setPivotAngle(ShooterPivotConstants.kAmpBottom);
              
  
            }
          }else {
            if(m_shooter.isPivotWithinTolerance(Rotation2d.fromDegrees(72), Rotation2d.fromDegrees(1)) && m_shooter.isWithinTolerance(1)){
              Logger.recordOutput("Starting amp TOP",true);
              kAmpTopReached = true;
            }

          }
          // if (m_drive.isTrajectoryComplete()){
          //     setRobotCurrentAction(RobotCurrentAction.kAmpShoot);
          // }
        }

        // update all subsystems
        // Pose2d curPose = m_drive.getPose();
        // Transform3d curTransformIntake = m_intake.getTransform();
        // Logger.recordOutput("RobotState/IntakePose", curTransformIntake);
        // Transform3d curTransformShooter = m_shooter.getTransform();
        // Logger.recordOutput("RobotState/ShooterPose", curTransformShooter);


    }

    public Pose2d getPredictedPose(double translationLookaheadS, double rotationLookaheadS) {
      return getEstimatedPose()
          .exp(
              new Twist2d(
                  robotVelocity.dx * translationLookaheadS,
                  robotVelocity.dy * translationLookaheadS,
                  robotVelocity.dtheta * rotationLookaheadS));
    }

    // public Command setToIntakePosition() {
    //     return Commands.runOnce(()-> {
    //         m_intake.setPivotAngle(IntakeConstants.kIntakeMinAngle);
    //         m_shooter.setPivotAngle(ShooterPivotConstants.homeAngle);
    //         m_intake.setIntakeSpeed(IntakeConstants.intakeSpeed);
    //     });
    // }

    

    public void calculateShooterAngle() {
        // calculate the shooter angle
        // ArrayList<Rotation2d> angles = m_shooterMath.getShooterAngle(new Pose3d(FieldConstants.kShooterCenter,new Rotation3d()),new Pose3d(m_drive.getPose()));
        // m_shooter.setPivotAngle(angles.get(1));
        // calculate shooter angle based on the robot pose if the velocity is held constant for .3 seconds
        ChassisSpeeds currentSpeeds = m_drive.getChassisSpeedsFieldRelative();
        Pose2d currentPose = m_drive.getPose();
        // add .3 seconds to the current pose
        Pose2d futurePose = new Pose2d(currentPose.getTranslation().plus(new Translation2d(currentSpeeds.vxMetersPerSecond*.3, currentSpeeds.vyMetersPerSecond*.3)), currentPose.getRotation());
        ArrayList<Rotation2d> vals = m_shooterMath.setNextShootingPoseAndVelocity(futurePose, new Twist2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond, currentSpeeds.omegaRadiansPerSecond), FieldConstants.kShooterCenter);
        Logger.recordOutput("Shoot angle", vals.get(1));
        // m_shooter.setPivotAngle(vals.get(1));
        

        // m_shooterMath.setNextShootingPoseAndVelocity(m_drive.getPose(), , new Translation3d(0,0,0));
    }

// under construction
     public SwerveTester getSwerveTester() {
        return m_swerveTester;
    }

    public ArrayList<AprilTag> getAprilTagsInView() {
        return new ArrayList<AprilTag>();
    }
    public void setAprilTagMap(AprilTagFieldLayout aprilTags) {
        // set the april tag map
    }
    @AutoLogOutput(key = "RobotState/EstimatedPose")
    public Pose2d getEstimatedPose() {
      return estimatedPose;
    }
    @AutoLogOutput(key = "RobotState/OdometryPose")
  public Pose2d getOdometryPose() {
    return odometryPose;
  }
}
