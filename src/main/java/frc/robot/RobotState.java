package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.hardwareprofiler.ProfilingScheduling;
import frc.lib.utils.SwerveTester;
import frc.robot.Constants.AmpConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.FlywheelConstants;
import frc.robot.Constants.ShooterConstants.ShooterPivotConstants;
import frc.robot.commands.autonomous.AutoFactory;
import frc.robot.commands.autonomous.CustomTrajectoryRunner;
import frc.robot.oi.DriverControls;
import frc.robot.oi.ManualController;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveProfiles;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.led.Led.LedState;
import frc.robot.subsystems.northstarAprilTagVision.AprilTagVision;
import frc.robot.subsystems.objectVision.ObjectDetectionCam;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterProfile;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.AutoBuilderManager;
import frc.robot.utils.CtreBaseRefreshManager;
import frc.robot.utils.DynamicCurrentLimitsManager;
import frc.robot.utils.NoteVisualizer;
import frc.robot.utils.ShooterMath;

public class RobotState {
  // singleton class that handles all inter subsystem communication
  private static RobotState instance = null;
  private Drive m_drive;
  private Climb m_climb;
  public Indexer m_indexer;
  public Shooter m_shooter;
  private ObjectDetectionCam m_objectDetectionCams;
  private AprilTagVision m_aprilTagVision;
  private Intake m_intake;
  private ShooterMath m_shooterMath;
  private SwerveTester m_swerveTester;
  private Amp m_amp;
  private Led m_led;
  private CustomTrajectoryRunner m_customTrajectoryRunner;
  private Command mDriveToPiece = null;
  private Pose2d mDriveToPiecePose = null;
  private Supplier<AutoFactory> mAutoFactory = null;
  private DriverControls mDriveControls;
  private AutoBuilderManager mAutoBuilderManager;
  private ManualController m_testingController;
  public Pose2d[] last3VisionUpdates = new Pose2d[1];
  public int visionNumber = 0;
  public double startedTryingToShoot = -1;
  public boolean m_autoForceShoot = false;

  public enum RobotCurrentAction {
    kStow, kIntake, kShootFender, kRevAndAlign, kShootWithAutoAlign, kAmpLineup, kAmpShoot, kAutoIntake, kAutoShoot,
    kPathPlanner, kVomit, kTune, kHockeyPuck, kGamePieceLock, kAutoFender, kNoteBackToIntake, kAutoSOTM, kDailedShot,
    kAutoShootAtPosition, kAutoRevAndAutoAlign, kSourceIntake, kNothing, kAutoAutoIntake, kAutoShootClose,kSourceShot
  }

  public RobotCurrentAction[] preRevActions = {
      RobotCurrentAction.kStow, RobotCurrentAction.kPathPlanner
  };

  public RobotCurrentAction[] noRevActions = {
      RobotCurrentAction.kIntake, RobotCurrentAction.kAutoIntake, RobotCurrentAction.kNoteBackToIntake,
  };

  public RobotCurrentAction[] activeShooting = {
      RobotCurrentAction.kShootWithAutoAlign, RobotCurrentAction.kShootFender, RobotCurrentAction.kAmpLineup,
      RobotCurrentAction.kAmpShoot, RobotCurrentAction.kAutoShoot, RobotCurrentAction.kTune,
      RobotCurrentAction.kHockeyPuck, RobotCurrentAction.kAutoFender, RobotCurrentAction.kAutoSOTM,
      RobotCurrentAction.kDailedShot, RobotCurrentAction.kAutoShootAtPosition, RobotCurrentAction.kAutoRevAndAutoAlign,
      RobotCurrentAction.kNothing, RobotCurrentAction.kRevAndAlign, RobotCurrentAction.kAutoShootClose,
      RobotCurrentAction.kHockeyPuck,RobotCurrentAction.kSourceIntake,RobotCurrentAction.kSourceShot
  };

  public Pose2d actualAutoShootAtPositionPose;

  public RobotCurrentAction curAction = RobotCurrentAction.kStow;
  public Timer stowAmpTimer;

  public Rotation2d shooterOverrideAngle = null;
  public long angleOverrideTime = 0;

  public double intakeStowTime = -1;
  public double shooterStopTime = -1;
  public double ledStopTime = -1;

  public boolean kAmpTopReached = false;
  public boolean mUpdatingAutoBuilder = false;
  public double kSourceIntakeStopTime = -1;

  public boolean lastInContactWithNote = true;

  public Pose2d stoppedPose = null;

  // auto intake timing stuff for auto
  public double lastTimeNoteSeen = -1;
  public double autoIntakeStartTime = -1;
  public Pose2d autoIntakeLastPose = new Pose2d();
  public int autoIntakeLeftOrRight = -1; // 1 is left -1 is right

  public record OdometryObservation(
      SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, double timestamp) {
  }

  public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
  }

  public enum GamePieceLocation {
    INTAKE,
    INDEXER,
    SHOOTER,
    NOT_IN_ROBOT
  }

  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
  private SwerveDriveWheelPositions lastWheelPositions = new SwerveDriveWheelPositions(
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

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer
      .createBuffer(poseBufferSizeSeconds);
  private Pose2d odometryPose = new Pose2d();
  private Pose2d estimatedPose = new Pose2d();
  public Pose2d currentTarget = new Pose2d();
  public LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<Command>("Auto Chooser");

  public Command m_driveToAmp = null;
  public Command m_driveToDailedShot = null;

  private RobotState(Drive drive, Climb climb, Indexer indexer, Shooter shooter, ObjectDetectionCam objectDetectionCams,
      AprilTagVision aprilTagVisions, Intake intake, Supplier<AutoFactory> autoFactory, DriverControls mDriverControls,
      Amp amp, Led led, AutoBuilderManager autoBuilderManager, ManualController m_testingController) {
    this.m_drive = drive;
    this.m_climb = climb;
    this.m_shooter = shooter;
    this.m_indexer = indexer;
    this.m_objectDetectionCams = objectDetectionCams;
    this.m_aprilTagVision = aprilTagVisions;
    this.m_intake = intake;
    this.mAutoFactory = autoFactory;
    this.mDriveControls = mDriverControls;
    this.m_amp = amp;
    this.m_led = led;
    this.mAutoBuilderManager = autoBuilderManager;
    this.m_testingController = m_testingController;
    NoteVisualizer.setRobotPoseSupplier(this::getEstimatedPose);
    m_shooterMath = new ShooterMath(new Translation3d(0, 0, Units.inchesToMeters(16.496)));
    ProfilingScheduling.startInstance();
    for (int i = 0; i < 3; ++i) {
      qStdDevs.set(i, 0, Math.pow(DriveConstants.odometryStateStdDevs.get(i, 0), 2));

    }

    m_customTrajectoryRunner = new CustomTrajectoryRunner();
    // this.m_swerveTester = new SwerveTester(drive);
  }

  public static RobotState startInstance(Drive drive, Climb climb, Indexer indexer, Shooter shooter,
      ObjectDetectionCam objectDetectionCams, AprilTagVision aprilTagVision, Intake intake,
      Supplier<AutoFactory> autoFactory, DriverControls mDriverControls, Amp amp, Led led,
      AutoBuilderManager autoBuilderManager, ManualController m_testingController) {
    if (instance == null) {

      instance = new RobotState(drive, climb, indexer, shooter, objectDetectionCams, aprilTagVision, intake,
          autoFactory, mDriverControls, amp, led, autoBuilderManager, m_testingController);

    }
    return instance;
  }

  public boolean hasNoteOverride() { // FOR TESTING ONLY
    return false;
    // return m_testingController.hasNoteOverride().getAsBoolean();
  }

  public static RobotState getInstance() {

    return instance;
  }

  public void setStoppedPosition(Pose2d pose) {
    stoppedPose = pose;
  }

  public Pose2d getStoppedPosition() {
    return stoppedPose;
  }

  public void resetAutoBuilder() {
    mAutoBuilderManager = new AutoBuilderManager();
  }

  public void setCurrentLimits(double moduleBudget, double shooterBudget, double shooterPivotBudget) {
    // m_drive.setModuleCurrentLimit(moduleBudget);
    // m_shooter.setFlywheelCurrentLimit(shooterBudget);
    // m_shooter.setPivotCurrentLimit(shooterPivotBudget);
  }

  public Pose2d[] getNotesInView() {
    return m_objectDetectionCams.getAllNotes();
  }

  public Pose2d getClosestNote() {
    return m_objectDetectionCams.getClosestNote();
  }

  public AutoBuilderManager getAutoBuilderManager() {
    return mAutoBuilderManager;
  }

  public ShooterMath getShooterMath() {
    return m_shooterMath;
  }

  public AutoFactory getAutoFactory() {
    return mAutoFactory.get();
  }

  public Drive getDrive() {
    return m_drive;
  }

  public boolean hasNote() {
    return m_indexer.inContactWithGamePiece();
  }

  public Transform3d getShooterTransform() {
    return m_shooter.getTransform();
  }

  // public ModuleLimits getModuleLimits(){
  // return ModuleConstants.freeSpeedLimits;
  // }

  public void setShooterSpeed(double speed) {
    m_shooter.setFlywheelSpeedWithSpin(speed, speed);
  }

  public void setIndexer(IndexerState state) {
    m_indexer.setState(state);

  }

  public void addVelocityData(Twist2d robotVelocity) {
    this.robotVelocity = robotVelocity;
    Logger.recordOutput("RobotVelocity/Velocty x", robotVelocity.dx);
    Logger.recordOutput("RobotVelocity/Velocty y", robotVelocity.dy);
    Logger.recordOutput("RobotVelocity/Velocty Theta", robotVelocity.dtheta);
  }

  public void setDriveType(DriveProfiles profile) {
    m_drive.setProfile(profile);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_drive.getChassisSpeeds();
  }

  public void setDriveToPieceChassisSpeeds(ChassisSpeeds speeds) {
    Logger.recordOutput("Drive to piece", speeds);
    m_drive.setDriveToPieceChassisSpeeds(speeds);
  }

  public void addVisionObservation(VisionObservation observation) {
    // latestParameters = null;
    // If measurement is old enough to be outside the pose buffer's timespan, skip.
    // if (edu.wpi.first.wpilibj.RobotState.isAutonomous()){
    // return ;
    // }
    // try {
    // if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSeconds
    // > observation.timestamp()) {
    // return;
    // }
    // } catch (NoSuchElementException ex) {
    // return;
    // }
    // // Get odometry based pose at timestamp
    // var sample = poseBuffer.getSample(observation.timestamp());
    // if (sample.isEmpty()) {
    // // exit if not there
    // System.out.println("Sample is empty");
    // return;
    // }

    // // sample --> odometryPose transform and backwards of that
    // var sampleToOdometryTransform = new Transform2d(getEstimatedPose(),
    // odometryPose);
    // var odometryToSampleTransform = new Transform2d(odometryPose,
    // getEstimatedPose());
    // // // get old estimate by applying odometryToSample Transform
    // Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

    // // Calculate 3 x 3 vision matrix
    // var r = new double[3];
    // for (int i = 0; i < 3; ++i) {
    // r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
    // }
    // // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // // and C = I. See wpimath/algorithms.md.
    // Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    // for (int row = 0; row < 3; ++row) {
    // double stdDev = qStdDevs.get(row, 0);
    // if (stdDev == 0.0) {
    // visionK.set(row, row, 0.0);
    // } else {
    // visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
    // }
    // }
    // difference between estimate and vision pose
    m_drive.addVisionMeasurement(observation);
      Logger.recordOutput("TimeStamps/visionUpdateNum", observation.timestamp());
      Logger.recordOutput("TimeStamps/realTime", Timer.getFPGATimestamp());
    // last3VisionUpdates[visionNumber] = observation.visionPose();
    visionNumber += 1;
    // if (visionNumber == 1) {
    //   visionNumber = 0;
    // }

    // System.out.println(observation.stdDevs());
    // Transform2d transform = new Transform2d(estimateAtTime,
    // observation.visionPose());
    // // scale transform by visionK
    // var kTimesTransform =
    // visionK.times(
    // VecBuilder.fill(
    // transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    // Transform2d scaledTransform =
    // new Transform2d(
    // kTimesTransform.get(0, 0),
    // kTimesTransform.get(1, 0),
    // Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

    // // Recalculate current estimate by applying scaled transform to old estimate
    // // then replaying odometry data
    // estimatedPose =
    // estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
  }

  public void goToShootPositionAndRev() {

  }

  public void setRobotCurrentAction(RobotCurrentAction action) {

    if (action == RobotCurrentAction.kAutoIntake) {
      mDriveToPiece = null;
    }
    if (action == RobotCurrentAction.kAmpLineup) {
      mDriveToPiece = null;
    }
    stowAmpTimer = null;

    if (action == RobotCurrentAction.kAutoShoot) {
      startedTryingToShoot = Timer.getFPGATimestamp();
    }

    for (int i = 0; i < preRevActions.length; i++) {
      if (preRevActions[i] == action) {
        m_shooter.setProfile(ShooterProfile.slowRev);
      }
    }
    for (int i = 0; i < activeShooting.length; i++) {
      if (activeShooting[i] == action) {
        m_shooter.setProfile(ShooterProfile.activeShooting);
      }
    }

    for (int i = 0; i < noRevActions.length; i++) {
      if (noRevActions[i] == action) {
        m_shooter.setProfile(ShooterProfile.notReving);
      }
    }
    if (curAction == RobotCurrentAction.kAutoIntake) {
      m_drive.setProfile(DriveProfiles.kDefault);
    }

    if (curAction == RobotCurrentAction.kAutoAutoIntake) {
      lastTimeNoteSeen = -1;
      autoIntakeStartTime = Timer.getFPGATimestamp();
    }
    // if(action == RobotCurrentAction.kStow){
    // if (curAction == RobotCurrentAction.kAutoIntake){
    // action = RobotCurrentAction.kPathPlanner;
    // }
    // }
    curAction = action;
  }

  /** Add odometry observation */
  public void addOdometryObservation(OdometryObservation observation) {

    Twist2d twist = DriveConstants.kDriveKinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
    Logger.recordOutput("Wheel 1 position", observation.wheelPositions().positions[0].distanceMeters);
    Logger.recordOutput("robot twist", twist);
    lastWheelPositions = observation.wheelPositions().copy();
    // Check gyro connected
    if (observation.gyroAngle != null) {
      // Update dtheta for twist if gyro connected
      twist = new Twist2d(
          twist.dx, twist.dy, observation.gyroAngle().minus(lastGyroAngle).getRadians());
      lastGyroAngle = observation.gyroAngle();
    }
    // Add twist to odometry pose
    odometryPose = odometryPose.exp(twist);
    // odometryPose = new
    // Pose2d(odometryPose.getTranslation(),observation.gyroAngle());
    // Add pose to buffer at timestamp
    poseBuffer.addSample(observation.timestamp(), odometryPose);
    // Calculate diff from last odometry pose and add onto pose estimate
    estimatedPose = estimatedPose.exp(twist);
  }

  public void saveActivePath(List<Pose2d> path) {
    activePath = path;
    if (path.size() > 0) {
      Logger.recordOutput("next target", path.get(path.size() - 1));
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
    Logger.recordOutput("target pose", pose);

  }

  public void setAutoShootOverride(Boolean forceShoot){
    m_autoForceShoot = forceShoot;
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
      Logger.recordOutput("stow Trigger 9", Timer.getFPGATimestamp());
      if (!edu.wpi.first.wpilibj.RobotState.isAutonomous()) {

        curAction = RobotCurrentAction.kStow;
      }
      stowShooter();
      stowAndStopIntake();
    }
    if (location == GamePieceLocation.SHOOTER) {
      if (!edu.wpi.first.wpilibj.RobotState.isTeleop()) {
        shooterStopTime = Timer.getFPGATimestamp() + 0.5;
        Logger.recordOutput("knows it shot", shooterStopTime);
      } else {
        shooterStopTime = Timer.getFPGATimestamp();
        Logger.recordOutput("knows it shot", shooterStopTime);
      }
      ;

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

  public Pose2d getDesiredRobotPose() {
    // return m_drive.getDesiredPose();
    return new Pose2d();
  }

  public void goToIntakePosition() {
    m_intake.setPivotAngle(IntakeConstants.kIntakeMinAngle);
    m_shooter.setPivotAngle(ShooterPivotConstants.homeAngle);
  }

  public void updateTestScheduler() {
    if (ProfilingScheduling.hasTests() && !DriverStation.isFMSAttached()) {
      ProfilingScheduling.updateCurrent();
    }
  }

  public Pose2d averageVisionPoses() {
    double x = 0;
    double y = 0;
    double t = 0;
    for (int a = 0; a < last3VisionUpdates.length; a++) {
      x += last3VisionUpdates[a].getX();
      y += last3VisionUpdates[a].getY();
      t += last3VisionUpdates[a].getRotation().getRadians();

    }
    x /= last3VisionUpdates.length;
    y /= last3VisionUpdates.length;
    t /= last3VisionUpdates.length;
    return new Pose2d(x, y, new Rotation2d(t));
  }

  public void manageAutoRev() {
    if (m_indexer.inContactWithGamePiece()) {
      m_shooter.setProfile(ShooterProfile.slowRev);
      double distance = m_shooterMath.getDistanceFromTarget(getEstimatedPose(),
          AllianceFlipUtil.apply(FieldConstants.kShooterCenter));
      if (distance > 5) {
        m_shooter.setProfile(ShooterProfile.notReving);
        return;
      }
      ArrayList<Rotation2d> headingAndPivotAngles = m_shooterMath.setNextShootingPoseAndVelocity(getEstimatedPose(),
          robotVelocity, AllianceFlipUtil.apply(FieldConstants.kShooterCenter));
      ArrayList<Double> shooterSpeeds = m_shooterMath.getShooterMetersPerSecond(distance);
      // m_shooter.setPivotAngle(headingAndPivotAngles.get(1));
      // m_shooter.setFlywheelSpeedWithSpin(shooterSpeeds.get(0), shooterSpeeds.get(1));
    } else {
      // m_shooter.setProfile(ShooterProfile.notReving);
    }
    // m_shooter.setProfile(ShooterProfile);
  }

  public void updateRobotState() {
    Logger.recordOutput("Current State Space", curAction);
    CtreBaseRefreshManager.getInstance().updateAll();
    DynamicCurrentLimitsManager.getInstance().update();
    if (mUpdatingAutoBuilder) {
      // mAutoBuilderManager.update();
    }
    if (intakeStowTime != -1 && intakeStowTime < Timer.getFPGATimestamp()) {
      intakeStowTime = -1;
    }
    if (shooterStopTime != -1 && shooterStopTime < Timer.getFPGATimestamp()) {
      if (!edu.wpi.first.wpilibj.RobotState.isAutonomous()) {
        m_shooter.setFlywheelSpeedWithSpin(ShooterConstants.FlywheelConstants.kIdleSpeed,
            ShooterConstants.FlywheelConstants.kIdleSpeed);
      }
      if (curAction != RobotCurrentAction.kAmpLineup) {
        if (!edu.wpi.first.wpilibj.RobotState.isAutonomous()) {
          // Logger.recordOutput("stow Trigger 8", Timer.getFPGATimestamp());
          // curAction = RobotCurrentAction.kStow;

        } else {
          if (curAction != RobotCurrentAction.kIntake) {
            if (curAction != RobotCurrentAction.kAutoAutoIntake) {
              m_drive.setProfile(DriveProfiles.kTrajectoryFollowing);
              setRobotCurrentAction(RobotCurrentAction.kPathPlanner);
            }
          }
        }
      }
      setGamePieceLocation(GamePieceLocation.NOT_IN_ROBOT);
      m_indexer.setState(IndexerState.IDLE);
      m_led.setState(LedState.NO_GAME_PIECE);
      stowShooter();
      shooterStopTime = -1;
      Logger.recordOutput("knows it shot", shooterStopTime);
    }

    if (ledStopTime != -1 && ledStopTime < Timer.getFPGATimestamp()) {
      m_led.setState(LedState.GAME_PIECE);
      ledStopTime = -1;
    }

    if (m_led.getState() != LedState.SHOOTER_READY && edu.wpi.first.wpilibj.RobotState.isEnabled()) {
      if (m_indexer.inContactWithGamePiece()) {
        if (m_led.getState() != LedState.FLASHING) {
          m_led.setState(LedState.GAME_PIECE);
        }
      } else {
        m_led.setState(LedState.NO_GAME_PIECE);
      }
    }

    if (curAction == RobotCurrentAction.kStow) {
      // m_drive.setProfile(DriveProfiles.kDefault);
      Logger.recordOutput("Is KStow", Timer.getFPGATimestamp());
      stowAndStopIntake();
      stowShooter();
      manageAutoRev();

      if (m_amp.getPivotAngle().getDegrees() > 5) {
        m_shooter.setPivotAngle(m_shooter.getPivotAngle());
        if (stowAmpTimer == null) {
          stowAmpTimer = new Timer();
          stowAmpTimer.start();
          m_intake.setPivotAngle(IntakeConstants.kAmpAngle.minus(Rotation2d.fromDegrees(10)));
        }

        if (stowAmpTimer.get() > .1) {
          m_amp.setPivotAngle(Rotation2d.fromDegrees(0));
          m_intake.setPivotAngle(IntakeConstants.kAmpAngle.minus(Rotation2d.fromDegrees(10)));

        }

        // Logger.recordOutput("intake angle",m_intake.getCurrentAngle().getDegrees());
        // Logger.recordOutput("intake angle >", IntakeConstants.kAmpAngle.getDegrees()
        // -5);
        // if (m_intake.getAngle().getDegrees() > IntakeConstants.kAmpAngle.getDegrees()
        // -5 ){
        // m_intake.setPivotAngle(IntakeConstants.kAmpAngle.minus(Rotation2d.fromDegrees(10)));

        // }else{
        // m_amp.setPivotAngle(Rotation2d.fromDegrees(0));
        // m_intake.setPivotAngle(IntakeConstants.kAmpAngle.minus(Rotation2d.fromDegrees(10)));
        // }
      }
      if (stowAmpTimer != null) {
        if (stowAmpTimer.get() > 3) {
          stowAmpTimer = null;
        }
      }
      // if(!edu.wpi.first.wpilibj.RobotState.isAutonomous()){
      // m_shooter.setFlywheelSpeedWithSpin(0,0);
      // }
      // m_indexer.setState(IndexerState.IDLE);
      // m_indexer.setState(IndexerState.IDLE);
    } else if (curAction == RobotCurrentAction.kTune) {
      m_indexer.setState(IndexerState.INDEXING);
    } else if (curAction == RobotCurrentAction.kIntake) {
      // m_drive.setProfile(DriveProfiles.kDefault);

      m_indexer.setState(IndexerState.INTAKING);

      m_intake.setIntakeSpeed(IntakeConstants.intakeSpeed);
      goToIntakePosition();
      m_shooter.setFlywheelSpeedWithSpin(0, 0);
    } else if (curAction == RobotCurrentAction.kSourceIntake) {
      // m_drive.setProfile(DriveProfiles.kDefault);

      m_indexer.setState(IndexerState.SOURCEINTAKING);

      // m_intake.setIntakeSpeed(IntakeConstants.intakeSpeed);

      m_shooter.setPivotAngle(Rotation2d.fromDegrees(ShooterPivotConstants.kSourceIntake.get()));
      m_shooter.setFlywheelSpeedWithSpin(FlywheelConstants.kSourceIntakeSpeed.get(),
          FlywheelConstants.kSourceIntakeSpeed.get());
    } else if (curAction == RobotCurrentAction.kNoteBackToIntake) {

      // m_intake.setIntakeSpeed(-IntakeConstants.intakeSpeed/3);
      // goToIntakePosition();
      m_shooter.setPivotAngle(Rotation2d.fromDegrees(13));
      m_shooter.setFlywheelSpeedWithSpin(0, 0);

    } else if (curAction == RobotCurrentAction.kShootFender) {
      m_shooter.setPivotAngle(ShooterPivotConstants.kFenderAngle);
      m_shooter.setFlywheelSpeedWithSpin(9, 9); // 9
      // m_indexer.setState(Indexer.IndexerState.SHOOTING);
      if (m_shooter.isWithinToleranceWithSpin(m_shooterMath.getShooterMetersPerSecond(2).get(0),
          m_shooterMath.getShooterMetersPerSecond(2).get(1), m_shooterMath.calculateAcceptableDropoff(2))
          && m_shooter.isPivotWithinTolerance(ShooterPivotConstants.kFenderAngle, Rotation2d.fromDegrees(5))) {
        m_indexer.setState(Indexer.IndexerState.SHOOTING);

      }

    } else if (curAction == RobotCurrentAction.kAutoShootClose) {
      m_shooter.setPivotAngle(Rotation2d.fromDegrees(ShooterPivotConstants.kCloseThreeAngle.get()));
      m_shooter.setFlywheelSpeedWithSpin(14, 9);
      m_indexer.m_fullSend = true;

    } else if (curAction == RobotCurrentAction.kRevAndAlign) {
      Pose2d predPose = getPredictedPose(0.07, 0.07);
      Translation3d finalTarget = AllianceFlipUtil.apply(frc.robot.FieldConstants.centerSpeakerOpening);

      ArrayList<Rotation2d> mRotations = m_shooterMath.setNextShootingPoseAndVelocity(predPose, robotVelocity,
          finalTarget);
      // double speed =
      // m_shooterMath.getShooterMetersPerSecond(m_shooterMath.getDistanceFromTarget(predPose,FieldConstants.kShooterCenter));

      m_drive.setProfile(DriveProfiles.kAutoAlign);

      double shootingDistance = m_shooterMath.getDistanceFromTarget(predPose, finalTarget);
      m_shooter.setFlywheelSpeedWithSpin(m_shooterMath.getShooterMetersPerSecond(shootingDistance).get(0),
          m_shooterMath.getShooterMetersPerSecond(shootingDistance).get(1));
      m_shooter.setPivotAngle(mRotations.get(1));
      m_drive.setDriveTurnOverride(mRotations.get(0));
      // ChassisSpeeds actualSpeed = m_drive.getChassisSpeeds();
      ArrayList<Double> speeds = (m_shooterMath.getShooterMetersPerSecond(shootingDistance));

      ChassisSpeeds actualSpeed = m_drive.getChassisSpeeds();
      ArrayList<Double> wheelyAmounts = m_drive.getWheelyAmounts();
      boolean flywheelInTolerance = m_shooter.isWithinToleranceWithSpin(speeds.get(0), speeds.get(1),
          m_shooterMath.calculateAcceptableDropoff(shootingDistance));
      boolean pivotInTolerance = (m_shooter.isPivotWithinTolerance(mRotations.get(1), Rotation2d.fromDegrees(1)));
      boolean headingWithinTolerance = (Math.abs(getEstimatedPose().getRotation().minus(mRotations.get(0))
          .getDegrees()) < m_shooterMath.calculateShootingHeadingTolerance(shootingDistance));
      boolean speedWithinTolerance = (Math.abs(actualSpeed.vxMetersPerSecond) < 1.
          && Math.abs(actualSpeed.vyMetersPerSecond) < 1. && Math.abs(actualSpeed.omegaRadiansPerSecond) < 0.2);
      // boolean withinVisionTolerance =
      // averageVisionPoses().getTranslation().getDistance(getEstimatedPose().getTranslation())
      // < Units.inchesToMeters(ShooterMathConstants.allowedDistance.get());
      boolean RollAndYawPerSecondIsSlow = (Math.abs(wheelyAmounts.get(0)) < .05
          && Math.abs(wheelyAmounts.get(1)) < .05);

      Logger.recordOutput("ReadyToShoot/FlywheelInTolerance", flywheelInTolerance);
      Logger.recordOutput("ReadyToShoot/PivotWithinTolerance", pivotInTolerance);
      Logger.recordOutput("ReadyToShoot/HeadingInTolerance", headingWithinTolerance);
      Logger.recordOutput("ReadyToShoot/SpeedWithinTolerance", speedWithinTolerance);
      // Logger.recordOutput("ReadyToShoot/withinVisionTolerance",
      // withinVisionTolerance);
      Logger.recordOutput("ReadyToShoot/HeadingTolerance",
          m_shooterMath.calculateShootingHeadingTolerance(shootingDistance));
      if (headingWithinTolerance && pivotInTolerance && speedWithinTolerance && flywheelInTolerance) {
        // if (mDriveControls.goToShootPositionAndRev().getAsBoolean()) {
        //   m_indexer.setState(Indexer.IndexerState.SHOOTING);
        //   m_led.setState(LedState.SHOOTER_READY);
        //   // if(Constants.getMode() == Mode.REPLAY){
        //   m_drive.onShootResetOdometryFocus();
        //   // }
        // }
      }
      // if
      // ((Math.abs(getEstimatedPose().getRotation().minus(mRotations.get(0)).getDegrees())
      // < DriveConstants.kShootToleranceDeg) &&
      // (m_shooter.isPivotWithinTolerance(mRotations.get(1),
      // Rotation2d.fromDegrees(1) )) && actualSpeed.vxMetersPerSecond < .05 &&
      // actualSpeed.vyMetersPerSecond < .05 && actualSpeed.omegaRadiansPerSecond < .1
      // && m_shooter.isWithinToleranceWithSpin(speeds.get(0),speeds.get(1)) ) {
      // // System.out.println("SHOULD BE RUMBLING");
      // // mDriveControls.setDriverRumble(0.2, RumbleType.kLeftRumble);

      // }
      //

    } else if (curAction == RobotCurrentAction.kAutoRevAndAutoAlign) {
      Pose2d predPose = getPredictedPose(0.07, 0.07);
      Translation3d finalTarget = AllianceFlipUtil.apply(frc.robot.FieldConstants.centerSpeakerOpening);

      ArrayList<Rotation2d> mRotations = m_shooterMath.setNextShootingPoseAndVelocity(predPose, robotVelocity,
          finalTarget);
      // double speed =
      // m_shooterMath.getShooterMetersPerSecond(m_shooterMath.getDistanceFromTarget(predPose,FieldConstants.kShooterCenter));

      m_drive.setProfile(DriveProfiles.kTrajectoryFollowing);

      double shootingDistance = m_shooterMath.getDistanceFromTarget(predPose, finalTarget);
      m_shooter.setFlywheelSpeedWithSpin(m_shooterMath.getShooterMetersPerSecond(shootingDistance).get(0),
          m_shooterMath.getShooterMetersPerSecond(shootingDistance).get(1));
      m_shooter.setPivotAngle(mRotations.get(1));
      m_drive.setDriveTurnOverride(mRotations.get(0));
      shooterOverrideAngle = mRotations.get(0);
      angleOverrideTime = System.currentTimeMillis() + 300;
      // ChassisSpeeds actualSpeed = m_drive.getChassisSpeeds();
      ArrayList<Double> speeds = (m_shooterMath.getShooterMetersPerSecond(shootingDistance));

      ChassisSpeeds actualSpeed = m_drive.getChassisSpeeds();
      ArrayList<Double> wheelyAmounts = m_drive.getWheelyAmounts();
      boolean flywheelInTolerance = m_shooter.isWithinToleranceWithSpin(speeds.get(0), speeds.get(1),
          m_shooterMath.calculateAcceptableDropoff(shootingDistance));
      boolean pivotInTolerance = (m_shooter.isPivotWithinTolerance(mRotations.get(1), Rotation2d.fromDegrees(1)));
      boolean headingWithinTolerance = (Math.abs(getEstimatedPose().getRotation().minus(mRotations.get(0))
          .getDegrees()) < m_shooterMath.calculateShootingHeadingTolerance(shootingDistance));
      boolean speedWithinTolerance = (Math.abs(actualSpeed.vxMetersPerSecond) < 1.
          && Math.abs(actualSpeed.vyMetersPerSecond) < 1. && Math.abs(actualSpeed.omegaRadiansPerSecond) < .2);
      boolean RollAndYawPerSecondIsSlow = (Math.abs(wheelyAmounts.get(0)) < .05
          && Math.abs(wheelyAmounts.get(1)) < .05);

      Logger.recordOutput("ReadyToShoot/FlywheelInTolerance", flywheelInTolerance);
      Logger.recordOutput("ReadyToShoot/PivotWithinTolerance", pivotInTolerance);
      Logger.recordOutput("ReadyToShoot/HeadingInTolerance", headingWithinTolerance);
      Logger.recordOutput("ReadyToShoot/SpeedWithinTolerance", speedWithinTolerance);

    } else if (curAction == RobotCurrentAction.kAutoShootAtPosition) {

      Pose2d predPose = actualAutoShootAtPositionPose;
      Logger.recordOutput("ReadyToShoot/Predicted Shooting Pose", predPose);
      Translation3d finalTarget = AllianceFlipUtil.apply(frc.robot.FieldConstants.centerSpeakerOpening);

      ArrayList<Rotation2d> mRotations = m_shooterMath.setNextShootingPoseAndVelocity(predPose, robotVelocity,
          finalTarget);
      // double speed =
      // m_shooterMath.getShooterMetersPerSecond(m_shooterMath.getDistanceFromTarget(predPose,FieldConstants.kShooterCenter));

      // m_drive.setProfile(DriveProfiles.kAutoPiecePickup);

      double shootingDistance = m_shooterMath.getDistanceFromTarget(predPose, finalTarget);
      m_shooter.setFlywheelSpeedWithSpin(m_shooterMath.getShooterMetersPerSecond(shootingDistance).get(0),
          m_shooterMath.getShooterMetersPerSecond(shootingDistance).get(1));
      m_shooter.setPivotAngle(mRotations.get(1));
      m_drive.setDriveTurnOverride(mRotations.get(0));
      // ChassisSpeeds actualSpeed = m_drive.getChassisSpeeds();
      ArrayList<Double> speeds = (m_shooterMath.getShooterMetersPerSecond(shootingDistance));

      ChassisSpeeds actualSpeed = m_drive.getChassisSpeeds();
      ArrayList<Double> wheelyAmounts = m_drive.getWheelyAmounts();
      boolean flywheelInTolerance = m_shooter.isWithinToleranceWithSpin(speeds.get(0), speeds.get(1),
          m_shooterMath.calculateAcceptableDropoff(shootingDistance));
      boolean pivotInTolerance = (m_shooter.isPivotWithinTolerance(mRotations.get(1), Rotation2d.fromDegrees(.5)));

      boolean headingWithinTolerance = (Math.abs(getEstimatedPose().getRotation().minus(mRotations.get(0))
          .getDegrees()) < m_shooterMath.calculateShootingHeadingTolerance(shootingDistance));
      boolean speedWithinTolerance = (Math.abs(actualSpeed.vxMetersPerSecond) < 1.
          && Math.abs(actualSpeed.vyMetersPerSecond) < 1. && Math.abs(actualSpeed.omegaRadiansPerSecond) < .2);
      boolean RollAndYawPerSecondIsSlow = (Math.abs(wheelyAmounts.get(0)) < .05
          && Math.abs(wheelyAmounts.get(1)) < .05);
      boolean atActualPose = (Math
          .abs(getEstimatedPose().getTranslation().getX() - predPose.getTranslation().getX()) < .3
          && Math.abs(getEstimatedPose().getTranslation().getY() - predPose.getTranslation().getY()) < .3);

      Logger.recordOutput("ReadyToShoot/FlywheelInTolerance", flywheelInTolerance);
      Logger.recordOutput("ReadyToShoot/PivotWithinTolerance", pivotInTolerance);
      Logger.recordOutput("ReadyToShoot/HeadingInTolerance", headingWithinTolerance);
      Logger.recordOutput("ReadyToShoot/HeadingOff",
          Math.abs(getEstimatedPose().getRotation().minus(mRotations.get(0)).getDegrees()));
      Logger.recordOutput("ReadyToShoot/SpeedWithinTolerance", speedWithinTolerance);
      Logger.recordOutput("ReadyToShoot/AtActualPose", atActualPose);
      if (Robot.isSimulation()) {
        flywheelInTolerance = true;
      }
      if (headingWithinTolerance && pivotInTolerance && speedWithinTolerance && flywheelInTolerance && atActualPose) {
        m_indexer.setState(Indexer.IndexerState.SHOOTING);

        mAutoBuilderManager.shot();
        setRobotCurrentAction(RobotCurrentAction.kStow);
      }
      // if
      // ((Math.abs(getEstimatedPose().getRotation().minus(mRotations.get(0)).getDegrees())
      // < DriveConstants.kShootToleranceDeg) &&
      // (m_shooter.isPivotWithinTolerance(mRotations.get(1),
      // Rotation2d.fromDegrees(1) )) && actualSpeed.vxMetersPerSecond < .05 &&
      // actualSpeed.vyMetersPerSecond < .05 && actualSpeed.omegaRadiansPerSecond < .1
      // && m_shooter.isWithinToleranceWithSpin(speeds.get(0),speeds.get(1)) ) {
      // // System.out.println("SHOULD BE RUMBLING");
      // // mDriveControls.setDriverRumble(0.2, RumbleType.kLeftRumble);

      // }
      //

    } else if (curAction == RobotCurrentAction.kDailedShot) {
      Pose2d predPose = frc.robot.FieldConstants.kDailedShot;

      Translation3d finalTarget = AllianceFlipUtil.apply(frc.robot.FieldConstants.centerSpeakerOpening);

      ArrayList<Rotation2d> mRotations = m_shooterMath.setNextShootingPoseAndVelocity(predPose, robotVelocity,
          finalTarget);
      // double speed =
      // m_shooterMath.getShooterMetersPerSecond(m_shooterMath.getDistanceFromTarget(predPose,FieldConstants.kShooterCenter));

      m_drive.setProfile(DriveProfiles.kAutoPiecePickup);

      double shootingDistance = m_shooterMath.getDistanceFromTarget(predPose, finalTarget);
      m_shooter.setFlywheelSpeedWithSpin(m_shooterMath.getShooterMetersPerSecond(shootingDistance).get(0),
          m_shooterMath.getShooterMetersPerSecond(shootingDistance).get(1));
      m_shooter.setPivotAngle(mRotations.get(1));
      m_drive.setDriveTurnOverride(mRotations.get(0));
      // ChassisSpeeds actualSpeed = m_drive.getChassisSpeeds();
      ArrayList<Double> speeds = (m_shooterMath.getShooterMetersPerSecond(shootingDistance));

      ChassisSpeeds actualSpeed = m_drive.getChassisSpeeds();
      boolean flywheelInTolerance = m_shooter.isWithinToleranceWithSpin(speeds.get(0), speeds.get(1),
          m_shooterMath.calculateAcceptableDropoff(shootingDistance));
      boolean pivotInTolerance = (m_shooter.isPivotWithinTolerance(mRotations.get(1), Rotation2d.fromDegrees(1)));
      boolean headingWithinTolerance = (Math.abs(
          getEstimatedPose().getRotation().minus(mRotations.get(0)).getDegrees()) < DriveConstants.kShootToleranceDeg);
      boolean speedWithinTolerance = (actualSpeed.vxMetersPerSecond < .5 && actualSpeed.vyMetersPerSecond < .5
          && actualSpeed.omegaRadiansPerSecond < .2);

      Logger.recordOutput("ReadyToShoot/FlywheelInTolerance", flywheelInTolerance);
      Logger.recordOutput("ReadyToShoot/PivotWithinTolerance", pivotInTolerance);
      Logger.recordOutput("ReadyToShoot/HeadingInTolerance", headingWithinTolerance);
      Logger.recordOutput("ReadyToShoot/SpeedWithinTolerance", speedWithinTolerance);
      if (headingWithinTolerance && pivotInTolerance && speedWithinTolerance && flywheelInTolerance) {
        m_indexer.setState(Indexer.IndexerState.SHOOTING);
      }
      // if
      // ((Math.abs(getEstimatedPose().getRotation().minus(mRotations.get(0)).getDegrees())
      // < DriveConstants.kShootToleranceDeg) &&
      // (m_shooter.isPivotWithinTolerance(mRotations.get(1),
      // Rotation2d.fromDegrees(1) )) && actualSpeed.vxMetersPerSecond < .05 &&
      // actualSpeed.vyMetersPerSecond < .05 && actualSpeed.omegaRadiansPerSecond < .1
      // && m_shooter.isWithinToleranceWithSpin(speeds.get(0),speeds.get(1)) ) {
      // // System.out.println("SHOULD BE RUMBLING");
      // // mDriveControls.setDriverRumble(0.2, RumbleType.kLeftRumble);

      // }
      //

    } else if (curAction == RobotCurrentAction.kShootWithAutoAlign) {
      Pose2d predPose = getPredictedPose(0.4, 0.4);
      Translation3d finalTarget = AllianceFlipUtil.apply(frc.robot.FieldConstants.centerSpeakerOpening);
      ArrayList<Rotation2d> mRotations = m_shooterMath.setNextShootingPoseAndVelocity(predPose, robotVelocity,
          finalTarget);
      // double speed =
      // m_shooterMath.getShooterMetersPerSecond(m_shooterMath.getDistanceFromTarget(predPose,FieldConstants.kShooterCenter));
      m_drive.setProfile(DriveProfiles.kAutoAlign);
      // m_shooter.setFlywheelSpeed(speed);
      m_shooter.setFlywheelSpeedWithSpin(
          m_shooterMath.getShooterMetersPerSecond(m_shooterMath.getDistanceFromTarget(predPose, finalTarget)).get(0),
          m_shooterMath.getShooterMetersPerSecond(m_shooterMath.getDistanceFromTarget(predPose, finalTarget)).get(1));
      m_shooter.setPivotAngle(mRotations.get(1));
      m_drive.setDriveTurnOverride(mRotations.get(0));
      if (Math.abs(
          getEstimatedPose().getRotation().minus(mRotations.get(0)).getDegrees()) < DriveConstants.kShootToleranceDeg) {
        m_indexer.setState(Indexer.IndexerState.SHOOTING);
      }
    } else if (curAction == RobotCurrentAction.kVomit) {
      m_intake.setIntakeSpeed(-1);
      goToIntakePosition();

    } else if (curAction == RobotCurrentAction.kAmpShoot) {
      Rotation2d finalShot = Rotation2d.fromDegrees(ShooterPivotConstants.kAmpShot.get());
      m_shooter.setPivotAngle(finalShot);
      m_shooter.setFlywheelSpeedWithSpin(FlywheelConstants.kAmpSpeed.get(), FlywheelConstants.kAmpSpeed.get());
      // if ((m_shooter.isPivotWithinTolerance(finalShot, Rotation2d.fromDegrees(1) ))
      // &&m_shooter.isWithinToleranceWithSpin(FlywheelConstants.kAmpSpeed.get(),FlywheelConstants.kAmpSpeed.get())
      // ) {
      // m_indexer.setState(Indexer.IndexerState.SHOOTING);
      // }
    } else if (curAction == RobotCurrentAction.kHockeyPuck) {
      Pose2d predPose = getPredictedPose(0.0, 0.0);
      Translation2d finalTarget = AllianceFlipUtil.apply(frc.robot.FieldConstants.kFeederAim);
      double distance = m_shooterMath.getDistanceFromTarget(predPose,
          new Translation3d(finalTarget.getX(), finalTarget.getY(), 0));
      ArrayList<Rotation2d> mRotations = m_shooterMath.setNextShootingPoseAndVelocityFeeder(predPose, robotVelocity,
          new Translation3d(finalTarget.getX(), finalTarget.getY(), 0.0));
      ArrayList<Double> mSpeeds = m_shooterMath.getShooterMetersPerSecondFeeder(distance);
      // double speed =
      // m_shooterMath.getShooterMetersPerSecond(m_shooterMath.getDistanceFromTarget(predPose,FieldConstants.kShooterCenter));

      m_drive.setProfile(DriveProfiles.kAutoAlign);

      m_shooter.setFlywheelSpeedWithSpin(mSpeeds.get(0), mSpeeds.get(1));
      m_shooter.setPivotAngle(mRotations.get(1));
      // m_shooter.setPivotAngle(mRotations.get(1));
      m_drive.setDriveTurnOverride(mRotations.get(0));
      ChassisSpeeds actualSpeed = m_drive.getChassisSpeeds();

    } else if (curAction == RobotCurrentAction.kSourceShot){
      Pose2d predPose = getPredictedPose(0.0, 0.0);
      Translation2d finalTarget = AllianceFlipUtil.apply(frc.robot.FieldConstants.kSourceMidShot);
      double distance = m_shooterMath.getDistanceFromTarget(predPose,
          new Translation3d(finalTarget.getX(), finalTarget.getY(), 0));
      ArrayList<Rotation2d> mRotations = m_shooterMath.setNextShootingPoseAndVelocityFeeder(predPose, robotVelocity,
          new Translation3d(finalTarget.getX(), finalTarget.getY(), 0.0));
      ArrayList<Double> mSpeeds = m_shooterMath.getShooterMetersPerSecondFeeder(distance);
      // double speed =
      // m_shooterMath.getShooterMetersPerSecond(m_shooterMath.getDistanceFromTarget(predPose,FieldConstants.kShooterCenter));

      m_drive.setProfile(DriveProfiles.kAutoAlign);

      m_shooter.setFlywheelSpeedWithSpin(mSpeeds.get(0), mSpeeds.get(1));
      m_shooter.setPivotAngle(mRotations.get(1));
      // m_shooter.setPivotAngle(mRotations.get(1));
      m_drive.setDriveTurnOverride(mRotations.get(0));
    }
    else if (curAction == RobotCurrentAction.kAmpLineup) {

      if (stowAmpTimer == null) {
        stowAmpTimer = new Timer();
        stowAmpTimer.start();
      }
      m_intake.setPivotAngle(IntakeConstants.kAmpAngle.minus(Rotation2d.fromDegrees(10)));
      if (stowAmpTimer.get() > .1) {
        m_amp.setPivotAngle(Rotation2d.fromDegrees(AmpConstants.kAmpShot.get()));
      }
      // DRIVE TO AMP
      // if(mDriveToPiece == null){
      // mDriveToPiece =
      // mAutoFactory.get().generateTrajectoryToPose(AllianceFlipUtil.apply(frc.robot.FieldConstants.kAmpBlue),DriveConstants.kAutoAlignToAmpSpeed,
      // false, instance);
      // mDriveToPiece.schedule();
      // }
      // m_drive.setProfile(DriveProfiles.kAutoPiecePickup);
      // m_drive.setDriveTurnOverride(AllianceFlipUtil.apply(frc.robot.FieldConstants.kAmpBlue).getRotation());

      // if(m_intake.getAngle().getDegrees() <
      // IntakeConstants.kAmpAngle.getDegrees()){

      // }
      m_shooter.setPivotAngle(Rotation2d.fromDegrees(ShooterPivotConstants.kAmpShot.get()));
      m_shooter.setFlywheelSpeedWithSpin(FlywheelConstants.kAmpSpeed.get(), FlywheelConstants.kAmpSpeed.get());
    }

    // if(kAmpTopReached == true){
    // m_indexer.setState(IndexerState.SHOOTING);

    // if(shooterStopTime != -1 && shooterStopTime < Timer.getFPGATimestamp()){

    // m_shooter.setPivotAngle(ShooterPivotConstants.kAmpBottom);

    // }
    // }else {
    // if(m_shooter.isPivotWithinTolerance(Rotation2d.fromDegrees(72),
    // Rotation2d.fromDegrees(1)) && m_shooter.isWithinTolerance(1)){
    // Logger.recordOutput("Starting amp TOP",true);
    // kAmpTopReached = true;
    // }

    // }
    // if (m_drive.isTrajectoryComplete()){
    // setRobotCurrentAction(RobotCurrentAction.kAmpShoot);
    // }

    else if (curAction == RobotCurrentAction.kPathPlanner) {
      if (getEstimatedPose().getTranslation().getDistance(FieldConstants.kshooterBase) < 3) {
        // m_shooter.setFlywheelSpeedWithSpin(ShooterConstants.FlywheelConstants.kIdleSpeedClose,ShooterConstants.FlywheelConstants.kIdleSpeedClose);
      } else {
        // m_shooter.setFlywheelSpeedWithSpin(ShooterConstants.FlywheelConstants.kIdleSpeedFar,ShooterConstants.FlywheelConstants.kIdleSpeedFar);
      }
      m_drive.setProfile(DriveProfiles.kTrajectoryFollowing);
      // m_indexer.setState(Indexer.IndexerState.INTAKING);

    } else if (curAction == RobotCurrentAction.kAutoShoot) {
      // first we stop
      // then we shoot
      // then we set to kPathPlanner

      m_drive.setProfile(DriveProfiles.kAutoShoot);
      Pose2d predPose = getPredictedPose(0.3, 0.3);
      Translation3d finalTarget = AllianceFlipUtil.apply(frc.robot.FieldConstants.centerSpeakerOpening);
      ArrayList<Rotation2d> mRotations = m_shooterMath.setNextShootingPoseAndVelocity(predPose, robotVelocity,
          finalTarget);
      double shootingDistance = m_shooterMath.getDistanceFromTarget(predPose, finalTarget);
      ArrayList<Double> speeds = m_shooterMath.getShooterMetersPerSecond(shootingDistance);
      m_shooter.setPivotAngle(mRotations.get(1));
      // m_drive.drive(new ChassisSpeeds(0, 0, 0));
      // m_shooter.setFlywheelSpeedWithSpin(speeds.get(0),speeds.get(1)); burblesquirp
      // and we da goats
      m_shooter.setFlywheelSpeedWithSpin(m_shooterMath.getShooterMetersPerSecond(shootingDistance).get(0),
          m_shooterMath.getShooterMetersPerSecond(shootingDistance).get(1));
      m_drive.setDriveTurnOverride(mRotations.get(0));

      ChassisSpeeds actualSpeed = m_drive.getChassisSpeeds();

      boolean flywheelInTolerance = m_shooter.isWithinToleranceWithSpin(speeds.get(0), speeds.get(1),
          m_shooterMath.calculateAcceptableDropoff(shootingDistance));
      boolean pivotInTolerance = (m_shooter.isPivotWithinTolerance(mRotations.get(1),
          Rotation2d.fromDegrees(m_shooterMath.calculateShootingPivotTolerance(shootingDistance))));

      boolean headingWithinTolerance = (Math.abs(getEstimatedPose().getRotation().minus(mRotations.get(0))
          .getDegrees()) < m_shooterMath.calculateShootingHeadingTolerance(shootingDistance));
      boolean speedWithinTolerance = (actualSpeed.vxMetersPerSecond < .5 && actualSpeed.vyMetersPerSecond < .5
          && actualSpeed.omegaRadiansPerSecond < .2);
      // boolean withinVisionTolerance =
      // averageVisionPoses().getTranslation().getDistance(getEstimatedPose().getTranslation())
      // < Units.inchesToMeters(ShooterMathConstants.allowedDistance.get()) ||
      // Timer.getFPGATimestamp() > startedTryingToShoot + 0.5;
      Logger.recordOutput("ReadyToShoot/FlywheelInTolerance", flywheelInTolerance);
      Logger.recordOutput("ReadyToShoot/PivotWithinTolerance", pivotInTolerance);
      Logger.recordOutput("ReadyToShoot/PivotTolerance",
          m_shooterMath.calculateShootingPivotTolerance(shootingDistance));
      Logger.recordOutput("ReadyToShoot/HeadingTolerance",
          m_shooterMath.calculateShootingHeadingTolerance(shootingDistance));
      Logger.recordOutput("ReadyToShoot/HeadingInTolerance", headingWithinTolerance);
      Logger.recordOutput("ReadyToShoot/SpeedWithinTolerance", speedWithinTolerance);
      // Logger.recordOutput("ReadyToShoot/VisionInTolerance",withinVisionTolerance);
      
      if (headingWithinTolerance && pivotInTolerance && speedWithinTolerance && flywheelInTolerance) {
        m_indexer.setState(Indexer.IndexerState.SHOOTING);
        RobotState.getInstance().setRobotCurrentAction(RobotCurrentAction.kPathPlanner);
      }
    } else if (curAction == RobotCurrentAction.kAutoSOTM) {
      Pose2d predPose = getPredictedPose(0, 0);
      Translation3d finalTarget = AllianceFlipUtil.apply(frc.robot.FieldConstants.centerSpeakerOpening);
      ArrayList<Rotation2d> mRotations = m_shooterMath.setNextShootingPoseAndVelocity(predPose, robotVelocity,
          finalTarget);
      ArrayList<Double> speeds = m_shooterMath
          .getShooterMetersPerSecond(m_shooterMath.getDistanceFromTarget(predPose, finalTarget));
      m_shooter.setPivotAngle(mRotations.get(1));
      // m_drive.drive(new ChassisSpeeds(0, 0, 0));
      // m_shooter.setFlywheelSpeedWithSpin(speeds.get(0),speeds.get(1));
      double shootingDistance = m_shooterMath.getDistanceFromTarget(predPose, finalTarget);
      m_shooter.setFlywheelSpeedWithSpin(
          m_shooterMath.getShooterMetersPerSecond(m_shooterMath.getDistanceFromTarget(predPose, finalTarget)).get(0),
          m_shooterMath.getShooterMetersPerSecond(m_shooterMath.getDistanceFromTarget(predPose, finalTarget)).get(1));
      // m_drive.setDriveTurnOverride(mRotations.get(0));
      ChassisSpeeds actualSpeed = m_drive.getChassisSpeeds();
      boolean flywheelInTolerance = m_shooter.isWithinToleranceWithSpin(speeds.get(0), speeds.get(1),
          m_shooterMath.calculateAcceptableDropoff(shootingDistance));
      boolean pivotInTolerance = (m_shooter.isPivotWithinTolerance(mRotations.get(1),
          Rotation2d.fromDegrees(m_shooterMath.calculateShootingPivotTolerance(shootingDistance))));
      boolean headingWithinTolerance = (Math.abs(getEstimatedPose().getRotation().minus(mRotations.get(0))
          .getDegrees()) < m_shooterMath.calculateShootingHeadingTolerance(shootingDistance));
      boolean speedWithinTolerance = (actualSpeed.vxMetersPerSecond < .5 && actualSpeed.vyMetersPerSecond < .5
          && actualSpeed.omegaRadiansPerSecond < .2);

      Logger.recordOutput("ReadyToShoot/FlywheelInTolerance", flywheelInTolerance);
      Logger.recordOutput("ReadyToShoot/PivotWithinTolerance", pivotInTolerance);
      Logger.recordOutput("ReadyToShoot/HeadingInTolerance", headingWithinTolerance);
      Logger.recordOutput("ReadyToShoot/SpeedWithinTolerance", speedWithinTolerance);

      shooterOverrideAngle = mRotations.get(0);
      angleOverrideTime = System.currentTimeMillis() + 1000;

      if (headingWithinTolerance && pivotInTolerance && speedWithinTolerance && flywheelInTolerance) {
        m_indexer.setState(Indexer.IndexerState.SHOOTING);
        setRobotCurrentAction(RobotCurrentAction.kIntake);

        // m_drive.setProfile(DriveProfiles.kTrajectoryFollowing);
        // setRobotCurrentAction(RobotCurrentAction.kPathPlanner);
      } else {
        // m_indexer.setState(Indexer.IndexerState.INDEXING);
      }

    } else if (curAction == RobotCurrentAction.kAutoShoot) {
      // first we stop
      // then we shoot
      // then we set to kPathPlanner

      m_drive.setProfile(DriveProfiles.kAutoShoot);
      Pose2d predPose = getPredictedPose(0.3, 0.3);
      Translation3d finalTarget = AllianceFlipUtil.apply(frc.robot.FieldConstants.centerSpeakerOpening);
      ArrayList<Rotation2d> mRotations = m_shooterMath.setNextShootingPoseAndVelocity(predPose, robotVelocity,
          finalTarget);
      double shootingDistance = m_shooterMath.getDistanceFromTarget(predPose, finalTarget);
      ArrayList<Double> speeds = m_shooterMath.getShooterMetersPerSecond(shootingDistance);
      m_shooter.setPivotAngle(mRotations.get(1));
      // m_drive.drive(new ChassisSpeeds(0, 0, 0));
      // m_shooter.setFlywheelSpeedWithSpin(speeds.get(0),speeds.get(1));
      m_shooter.setFlywheelSpeedWithSpin(m_shooterMath.getShooterMetersPerSecond(shootingDistance).get(0),
          m_shooterMath.getShooterMetersPerSecond(shootingDistance).get(1));
      m_drive.setDriveTurnOverride(mRotations.get(0));

      ChassisSpeeds actualSpeed = m_drive.getChassisSpeeds();

      boolean flywheelInTolerance = m_shooter.isWithinToleranceWithSpin(speeds.get(0), speeds.get(1), 1);
      boolean pivotInTolerance = (m_shooter.isPivotWithinTolerance(mRotations.get(1), Rotation2d.fromDegrees(1)));
      boolean headingWithinTolerance = (Math.abs(getEstimatedPose().getRotation().minus(mRotations.get(0))
          .getDegrees()) < m_shooterMath.calculateShootingHeadingTolerance(shootingDistance));
      boolean speedWithinTolerance = (actualSpeed.vxMetersPerSecond < .5 && actualSpeed.vyMetersPerSecond < .5
          && actualSpeed.omegaRadiansPerSecond < .2);

      Logger.recordOutput("ReadyToShoot/FlywheelInTolerance", flywheelInTolerance);
      Logger.recordOutput("ReadyToShoot/PivotWithinTolerance", pivotInTolerance);
      Logger.recordOutput("ReadyToShoot/HeadingInTolerance", headingWithinTolerance);
      Logger.recordOutput("ReadyToShoot/SpeedWithinTolerance", speedWithinTolerance);
      if (headingWithinTolerance && pivotInTolerance && speedWithinTolerance && flywheelInTolerance) {
        m_indexer.setState(Indexer.IndexerState.SHOOTING);
      }
    } else if (curAction == RobotCurrentAction.kAutoAutoIntake) {
      m_intake.setIntakeSpeed(IntakeConstants.intakeSpeed);
      m_indexer.setState(IndexerState.INTAKING);
      goToIntakePosition();
      m_shooter.setFlywheelSpeedWithSpin(0, 0);
      Pose2d closestNote = m_objectDetectionCams.getClosestNote();
      if (closestNote != null) {
        if (AllianceFlipUtil.apply(closestNote).getX() > 9.) {
          closestNote = null;
        }
      }
      if (m_indexer.inContactWithGamePiece()) {
        Logger.recordOutput("AutoAutoIntake/End", Timer.getFPGATimestamp());
        setRobotCurrentAction(RobotCurrentAction.kPathPlanner);
        m_drive.setProfile(DriveProfiles.kTrajectoryFollowing);
        if (AllianceFlipUtil.apply(getEstimatedPose().getX()) > 3) {
          if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
            m_drive.driveAuto(ChassisSpeeds.fromFieldRelativeSpeeds(5, 0, 0, getEstimatedPose().getRotation()));

          } else {
            m_drive.driveAuto(ChassisSpeeds.fromFieldRelativeSpeeds(-5, 0, 0, getEstimatedPose().getRotation()));
          }
        }
        return;
      }
      Logger.recordOutput("closest note", closestNote);
      // if (closestNote == null && mDriveToPiece == null) {
      // m_drive.setProfile(DriveProfiles.kTrajectoryFollowing);
      // }
      if (closestNote == null) {
        if (lastTimeNoteSeen + 0.1 < Timer.getFPGATimestamp()) {
          // m_drive.setProfile(DriveProfiles.kTrajectoryFollowing);
          m_drive.setProfile(DriveProfiles.kDefault);
          // if(Math.abs(getEstimatedPose().getRotation().minus(Rotation2d.fromDegrees(0)).getDegrees())<90){
          // if(DriverStation.getAlliance().get().equals(Alliance.Blue)){
            if(DriverStation.getAlliance().get().equals(Alliance.Red)){
          m_drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, autoIntakeLeftOrRight * -1,
              Rotation2d.fromDegrees(autoIntakeLeftOrRight * 90).getRadians(), getEstimatedPose().getRotation()));
            }else{
              m_drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, autoIntakeLeftOrRight * 1,
              Rotation2d.fromDegrees(autoIntakeLeftOrRight * 90).getRadians(), getEstimatedPose().getRotation()));
            }
          // } else {
          // m_drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.0,
          // autoIntakeLeftOrRight *1, Rotation2d.fromDegrees(autoIntakeLeftOrRight
          // *90).getRadians(),getEstimatedPose().getRotation()));
          // }
          // }else{
          // m_drive.drive(new ChassisSpeeds(-1, 0, 0));
          // }
          // if(getEstimatedPose())
        }
        return;
      }

      // add rotational override to point the robot the robot in the direction of the
      // piece
      // make sure the back of the robot is facing the piece
      mDriveToPiecePose = closestNote;
      m_drive.setProfile(DriveProfiles.kAutoPiecePickup);
      Translation2d robotToPiece = m_drive.getPose().getTranslation().minus(mDriveToPiecePose.getTranslation());
      Rotation2d robotToPieceRot = robotToPiece.getAngle();
      m_drive.setDriveTurnOverride(robotToPieceRot);
      m_drive.setDriveToPieceChassisSpeeds(DriveConstants.holonomicDrive.calculate(getEstimatedPose(), closestNote));
      if (m_intake.hasNote()) {
        m_drive.setDriveToPieceChassisSpeeds(new ChassisSpeeds());
        // setRobotCurrentAction(RobotCurrentAction.kPathPlanner);
        // m_drive.setProfile(DriveProfiles.kTrajectoryFollowing);

      }
      Logger.recordOutput("Turn angle", robotToPiece.getAngle().minus(getEstimatedPose().getRotation()));
      // Rotation2d.fromRadians(Math.tan(robotToPiece.getX()/robotToPiece.getY())).plus(Rotation2d.fromDegrees(180));

      shooterOverrideAngle = robotToPieceRot;
      // angleOverrideTime = System.currentTimeMillis() + 1000;
      lastTimeNoteSeen = Timer.getFPGATimestamp();
      // m_drive.setDriveTurnOverride(robotToPieceRot);

      // if(mDriveToPiecePose.getTranslation().getDistance(m_objectDetectionCams.getClosestNote().getTranslation())
      // > Units.inchesToMeters(12)){
      // mDriveToPiece = null;
      // }
    }

    else if (curAction == RobotCurrentAction.kAutoFender) {
      // first we stop
      // then we shoot
      // then we set to kPathPlanner

      // m_drive.setProfile(DriveProfiles.kAutoShoot);
      Pose2d predPose = getPredictedPose(0.3, 0.3);
      Translation3d finalTarget = AllianceFlipUtil.apply(frc.robot.FieldConstants.centerSpeakerOpening);
      ArrayList<Rotation2d> mRotations = m_shooterMath.setNextShootingPoseAndVelocity(predPose, robotVelocity,
          finalTarget);
      ArrayList<Double> speeds = m_shooterMath
          .getShooterMetersPerSecond(m_shooterMath.getDistanceFromTarget(predPose, finalTarget));
      m_shooter.setPivotAngle(ShooterPivotConstants.kFenderAngle);
      m_drive.drive(new ChassisSpeeds(0, 0, 0));
      // m_shooter.setFlywheelSpeedWithSpin(speeds.get(0),speeds.get(1));
      m_shooter.setFlywheelSpeedWithSpin(10, 10);
      // m_drive.setDriveTurnOverride(mRotations.get(0));
      Logger.recordOutput("PLS", Math.abs(
          getEstimatedPose().getRotation().minus(mRotations.get(0)).getDegrees()) < DriveConstants.kShootToleranceDeg);
      Logger.recordOutput("Second", (m_shooter.isPivotWithinTolerance(mRotations.get(1), Rotation2d.fromDegrees(3))));
      ChassisSpeeds actualSpeed = m_drive.getChassisSpeeds();
      if ((m_shooter.isPivotWithinTolerance(mRotations.get(1), Rotation2d.fromDegrees(6)))
          && m_shooter.isWithinToleranceWithSpin(10, 10, 3.)) {
        m_indexer.setState(Indexer.IndexerState.SHOOTING);
        // m_drive.setProfile(DriveProfiles.kTrajectoryFollowing);
        setRobotCurrentAction(RobotCurrentAction.kPathPlanner);
      }

    } else if (curAction == RobotCurrentAction.kGamePieceLock) {
      m_drive.setProfile(DriveProfiles.kAutoAlign);
      Pose2d predictedPose = getPredictedPose(0, 0);
      Pose2d closestNote = m_objectDetectionCams.getClosestNote();
      if (closestNote != null) {

        ArrayList<Rotation2d> mRotations = m_shooterMath.setNextShootingPoseAndVelocity(predictedPose, new Twist2d(),
            new Translation3d(closestNote.getTranslation().getX(), closestNote.getTranslation().getY(), 0));
        m_drive.setDriveTurnOverride(mRotations.get(0));
      }

    } else if (curAction == RobotCurrentAction.kAutoIntake) {

      m_intake.setIntakeSpeed(IntakeConstants.intakeSpeed);
      m_indexer.setState(IndexerState.INTAKING);
      goToIntakePosition();
      m_shooter.setFlywheelSpeedWithSpin(0, 0);
      Pose2d closestNote = m_objectDetectionCams.getClosestNote();

      if (m_indexer.inContactWithGamePiece()) {
        // setRobotCurrentAction(RobotCurrentAction.kStow);
        m_drive.setProfile(DriveProfiles.kDefault);
        return;
      }
      Logger.recordOutput("closest note", closestNote);
      // if (closestNote == null && mDriveToPiece == null) {
      // m_drive.setProfile(DriveProfiles.kTrajectoryFollowing);
      // }
      if (closestNote == null) {
        if (lastTimeNoteSeen + 0.1 < Timer.getFPGATimestamp()) {
          m_drive.setProfile(DriveProfiles.kDefault);
        }
        return;
      }
      // if(closestNote.getTranslation().getDistance(getEstimatedPose().getTranslation())
      // < 1.0 && m_intake.getAngle().getDegrees() > -10){
      // m_drive.setProfile(DriveProfiles.kDefault);
      // m_drive.drive(new ChassisSpeeds());
      // return;
      // }

      // add rotational override to point the robot the robot in the direction of the
      // piece
      // make sure the back of the robot is facing the piece
      mDriveToPiecePose = closestNote;
      m_drive.setProfile(DriveProfiles.kAutoPiecePickup);
      Translation2d robotToPiece = m_drive.getPose().getTranslation().minus(mDriveToPiecePose.getTranslation());
      Rotation2d robotToPieceRot = robotToPiece.getAngle();
      m_drive.setDriveTurnOverride(robotToPieceRot);
      m_drive.setDriveToPieceChassisSpeeds(DriveConstants.holonomicDrive.calculate(getEstimatedPose(), closestNote));
      m_led.setState(LedState.AUTO_DRIVING_TO_NOTE);
      if (m_intake.hasNote()) {
        m_drive.setDriveToPieceChassisSpeeds(new ChassisSpeeds());
        // setRobotCurrentAction(RobotCurrentAction.kStow);
        m_drive.setProfile(DriveProfiles.kDefault);
      }
      Logger.recordOutput("Turn angle", robotToPiece.getAngle().minus(getEstimatedPose().getRotation()));
      // Rotation2d.fromRadians(Math.tan(robotToPiece.getX()/robotToPiece.getY())).plus(Rotation2d.fromDegrees(180));

      shooterOverrideAngle = robotToPieceRot;
      // angleOverrideTime = System.currentTimeMillis() + 1000;
      lastTimeNoteSeen = Timer.getFPGATimestamp();
      // m_drive.setDriveTurnOverride(robotToPieceRot);

      // if(mDriveToPiecePose.getTranslation().getDistance(m_objectDetectionCams.getClosestNote().getTranslation())
      // > Units.inchesToMeters(12)){
      // mDriveToPiece = null;
      // }

    }

    // update all subsystems
    Pose2d curPose = m_drive.getPose();
    Transform3d curTransformIntake = m_intake.getTransform();
    Logger.recordOutput("RobotState/IntakePose", curTransformIntake);
    Transform3d curTransformShooter = m_shooter.getTransform();
    Logger.recordOutput("RobotState/ShooterPose", curTransformShooter);

    if (m_indexer.inContactWithGamePiece() != lastInContactWithNote) {
      lastInContactWithNote = !lastInContactWithNote;
      Commands.runOnce(() -> {
        Logger.recordOutput("Rumble", 0.1);
        setDriverRumble(0.1, RumbleType.kBothRumble);
      }).andThen(Commands.waitSeconds(1.5)).andThen(Commands.runOnce(() -> {
        Logger.recordOutput("Rumble", 0);
        setDriverRumble(0., RumbleType.kBothRumble);
      })).schedule();
      if (m_indexer.inContactWithGamePiece() && edu.wpi.first.wpilibj.RobotState.isEnabled()) {
        m_led.setState(LedState.FLASHING);
        ledStopTime = Timer.getFPGATimestamp() + 1;
      }
    }

  }

  public void setDriverRumble(double value, RumbleType side) {
    mDriveControls.setDriverRumble(value, side);
    m_testingController.setRumble(value, side);
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
  // return Commands.runOnce(()-> {
  // m_intake.setPivotAngle(IntakeConstants.kIntakeMinAngle);
  // m_shooter.setPivotAngle(ShooterPivotConstants.homeAngle);
  // m_intake.setIntakeSpeed(IntakeConstants.intakeSpeed);
  // });
  // }

  public void calculateShooterAngle() {
    // calculate the shooter angle
    // ArrayList<Rotation2d> angles = m_shooterMath.getShooterAngle(new
    // Pose3d(FieldConstants.kShooterCenter,new Rotation3d()),new
    // Pose3d(m_drive.getPose()));
    // m_shooter.setPivotAngle(angles.get(1));
    // calculate shooter angle based on the robot pose if the velocity is held
    // constant for .3 seconds
    ChassisSpeeds currentSpeeds = m_drive.getChassisSpeedsFieldRelative();
    Pose2d currentPose = m_drive.getPose();
    // add .3 seconds to the current pose
    Pose2d futurePose = new Pose2d(
        currentPose.getTranslation()
            .plus(new Translation2d(currentSpeeds.vxMetersPerSecond * .3, currentSpeeds.vyMetersPerSecond * .3)),
        currentPose.getRotation());
    ArrayList<Rotation2d> vals = m_shooterMath.setNextShootingPoseAndVelocity(futurePose,
        new Twist2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond,
            currentSpeeds.omegaRadiansPerSecond),
        FieldConstants.kShooterCenter);
    Logger.recordOutput("Shoot angle", vals.get(1));
    // m_shooter.setPivotAngle(vals.get(1));

    // m_shooterMath.setNextShootingPoseAndVelocity(m_drive.getPose(), , new
    // Translation3d(0,0,0));
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
    return m_drive.getPose();
  }

  // @AutoLogOutput(key= "RobotState/PoseTimeAgo")
  public Pose2d getPoseTimeAgo(double time) {
    return m_drive.getPoseTimeAgo(time);
  }

  @AutoLogOutput(key = "RobotState/OdometryPose")
  public Pose2d getOdometryPose() {
    return m_drive.getPose();
  }
}
