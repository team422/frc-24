package frc.robot.subsystems.drive;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.IntStream;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.hardwareprofiler.DataPoint;
import frc.lib.hardwareprofiler.HardwareProfiler;
import frc.lib.hardwareprofiler.PowerConsumptionHelper;
import frc.lib.hardwareprofiler.ProfiledSubsystem;
import frc.lib.hardwareprofiler.ProfilingScheduling;
import frc.lib.utils.CustomHolmonomicDrive;
import frc.lib.utils.FieldUtil;
import frc.lib.utils.LoggedTunableNumber;
import frc.lib.utils.PoseEstimator;
import frc.lib.utils.PoseEstimator.TimestampedVisionUpdate;
import frc.lib.utils.SubsystemProfiles;
import frc.lib.utils.SwerveModuleVoltages;
import frc.lib.utils.SwerveTester;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroInputsAutoLogged;
import frc.robot.utils.EqualsUtil;
import frc.robot.utils.swerve.ModuleLimits;
import frc.robot.utils.swerve.SwerveSetpoint;
import frc.robot.utils.swerve.SwerveSetpointGenerator;

public class Drive extends ProfiledSubsystem {
  private final SwerveModuleIO[] m_modules;
  private final SwerveModuleInputsAutoLogged[] m_inputs;

  private final SwerveDrivePoseEstimator m_poseEstimator;

  private final SwerveSetpointGenerator setpointGenerator;

  private final GyroIO m_gyro;
  private final GyroInputsAutoLogged m_gyroInputs;

  private final double[] m_lockAngles = new double[] { 45, 315, 45, 315 };
  private boolean m_hasResetOdometry;
  private double m_simGyroLastUpdated;
  private double m_lastFPGATimestamp;

  public ChassisSpeeds m_desChassisSpeeds;
  public ChassisSpeeds m_desAutoChassisSpeeds;
  public ChassisSpeeds m_driveToPieceSpeeds;
  public ChassisSpeeds m_autoShootChassisSpeeds = new ChassisSpeeds(0, 0, 0);
  public SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(ModuleConstants.kffkV.get(), ModuleConstants.kffkV.get(),ModuleConstants.kffkA.get());
  public PIDController m_driveController;
  public PIDController m_turnController;
  public PIDController m_driveFFPIDController;
  private double[] lastModulePositionsMeters = new double[] { 0.0, 0.0, 0.0, 0.0 };

  private Rotation2d turnOverride = new Rotation2d();

  private SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(ModuleConstants.kffkV.get(), ModuleConstants.kffkV.get(),ModuleConstants.kffkA.get());

  private int[] m_moduleNumbers = {0, 1, 2, 3};
  private SubsystemProfiles m_profiles;

  public CustomHolmonomicDrive m_holonomicController;

  public Integer m_activeWheel;
  private SwerveDriveWheelPositions lastPositions = null;
  

  @AutoLog
  public static class OdometryTimestampInputs {
    public double[] timestamps = new double[] {};
  }  
  private double lastTime = 0.0;

  public static final Lock odometryLock = new ReentrantLock();
  // TODO: DO THIS BETTER!
  public static final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(100);

  private final OdometryTimestampInputsAutoLogged odometryTimestampInputs =
  new OdometryTimestampInputsAutoLogged();

  public boolean setChassisSpeedsDirectionPIDOnly = false;
  public boolean m_singleWheelDriveMode = false;

  public enum DriveProfilingSuite {
    kSetpointTime, kSetpointDeltaAtTime, kNone, kResetting, kWaiting, kFeedForwardAccuracy
  }

  public enum DriveProfiles {
    kDefault, kTuning, kTesting, kFFdrive, kFFPIDDrive, kModuleAndAccuracyTesting, kTrajectoryFollowing, kAutoAlign, kShootWithTrajectory,kAutoPiecePickup,kAutoShoot,WHEEL_RADIUS_CHARACTERIZATION,CHARACTERIZATION
  }

  // Profiling variables
  public DriveProfilingSuite m_currentProfileTest = DriveProfilingSuite.kNone;
  public DriveProfilingSuite m_lastProfileTest = DriveProfilingSuite.kNone;
  public int m_profileTestIndex = 0;
  public HardwareProfiler m_profiler = null;
  public HardwareProfiler m_profiler2 = null;
  public PowerConsumptionHelper testPowerConsumption = null;
  public boolean m_profileTestRunning = false;
  public Double m_testStartTime = null;
  public SwerveModuleVoltages m_moduleVoltageState;

  public Boolean m_allowCameraOdometeryConnections = true;

  public Boolean m_createdAprilTagFieldLayout = false;
  public Boolean m_creatingAprilTagFieldLayout = true;

  public boolean m_moduleAndAccuracyTestRunning = false;
  public SwerveModuleState m_moduleState = new SwerveModuleState(0, new Rotation2d(0));
  double[] wheelSpeedsLikelyhood;
  double[] wheelSpeedsCorrection;

  private PoseEstimator poseEstimator = new PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002));
  public SwerveTester m_swerveTester;

private SwerveSetpoint currentSetpoint =
      new SwerveSetpoint(
          new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
          });
  public double characterizationInput = 0 ;

  public boolean modulesOrienting = false;

  public enum AprilTagFieldCreationStage {
    kInit, kSpinningToFind, kMoveToTag, kMoveBetweenTags
  }

  public AprilTagFieldCreationStage m_aprilTagFieldCreationStage = AprilTagFieldCreationStage.kSpinningToFind;

  // public SwerveDrivePoseEstimator m_DrivePoseEstimator;

  public ArrayList<AprilTag> m_aprilTagsFoundAsTransforms = new ArrayList<AprilTag>();
  public ArrayList<Integer> m_tags = new ArrayList<Integer>();
  private Rotation2d lastGyroYaw = new Rotation2d();
  Rotation2d m_aprilTagFieldRotation = new Rotation2d(0);

  PIDController m_autoAlignController = new PIDController(3.5, 0.0, 0.09);

  private static final LoggedTunableNumber coastSpeedLimit =
  new LoggedTunableNumber(
          "Drive/CoastSpeedLimit", DriveConstants.kMaxSpeedMetersPerSecond * 0.6);
  private static final LoggedTunableNumber coastDisableTime =
    new LoggedTunableNumber("Drive/CoastDisableTimeSeconds", 0.5);


  /** Creates a new Drive. */
  public Drive(GyroIO gyro, Pose2d startPose, SwerveModuleIO... modules) {
    m_modules = modules;
    m_gyro = gyro;
    m_gyroInputs = new GyroInputsAutoLogged();
    setpointGenerator =
        SwerveSetpointGenerator.builder()
            .kinematics(DriveConstants.kDriveKinematics)
            .moduleLocations(DriveConstants.kModuleTranslations)
            .build();
    for (int i = 0; i < 10; i++) {
      for (SwerveModuleIO module : m_modules) {
        module.resetDistance();
        module.syncTurningEncoder();
        // module.resetEncoders();
      }
    }
    
    m_inputs = new SwerveModuleInputsAutoLogged[modules.length];
    for (int i = 0; i < m_inputs.length; i++) {
      m_inputs[i] = new SwerveModuleInputsAutoLogged();
    }
    m_poseEstimator = new SwerveDrivePoseEstimator(
      Constants.DriveConstants.kDriveKinematics, m_gyro.getAngle(), getSwerveModulePositions(), startPose);
      m_hasResetOdometry = false;
      m_lastFPGATimestamp = Timer.getFPGATimestamp();
      
      HashMap<Enum<?>, Runnable> drivePeriodicHash = new HashMap<Enum<?>, Runnable>();
      drivePeriodicHash.put(DriveProfiles.kDefault, this::defaultPeriodic);
      drivePeriodicHash.put(DriveProfiles.kTuning, this::tuningPeriodic);
      
      drivePeriodicHash.put(DriveProfiles.kTesting, this::testingPeriodic);
      drivePeriodicHash.put(DriveProfiles.kFFdrive, this::ffPeriodic);
      drivePeriodicHash.put(DriveProfiles.kModuleAndAccuracyTesting, this::moduleAndAccuracyTesting);
      drivePeriodicHash.put(DriveProfiles.kTrajectoryFollowing, this::trajectoryFollowingPeriodic);
      Class<? extends Enum<?>> profileEnumClass = DriveProfiles.class;
      Enum<?> defaultProfile = DriveProfiles.kDefault;
      m_profiles = new SubsystemProfiles(profileEnumClass, drivePeriodicHash, defaultProfile);
      ff = new SimpleMotorFeedforward(ModuleConstants.kffkS.get(), ModuleConstants.kffkV.get(), 0);
      for (SwerveModuleIO module : m_modules) {
        module.setDrivePID(ModuleConstants.kDriveP.get(), ModuleConstants.kDriveI.get(), ModuleConstants.kDriveD.get());
        module.setTurnPID(ModuleConstants.FLkTurningP.get(), ModuleConstants.FLkTurningI.get(), ModuleConstants.FLkTurningD.get());
      }
      // m_DrivePoseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, m_gyro.getAngle(), getSwerveModulePositions(), startPose);
      
    this.m_driveFeedforward = new SimpleMotorFeedforward(ModuleConstants.kDriveKS.get(), ModuleConstants.kDriveKV.get(),
        ModuleConstants.kDriveKA.get());
    this.m_driveController = new PIDController(ModuleConstants.kDriveP.get(), ModuleConstants.kDriveI.get(),
        ModuleConstants.kDriveD.get());
    if (Robot.isReal()) {
      this.m_turnController = new PIDController(ModuleConstants.FLkTurningP.get(), ModuleConstants.FLkTurningI.get(),
          ModuleConstants.FLkTurningD.get());
    } else {
      this.m_turnController = new PIDController(ModuleConstants.kTurningPSim.get(), ModuleConstants.kTurningISim.get(),
          ModuleConstants.kTurningDSim.get());
    }
    this.m_driveFFPIDController = new PIDController(ModuleConstants.kFFDriveP.get(), ModuleConstants.kFFDriveI.get(),
        ModuleConstants.kFFDriveD.get());
    m_holonomicController = DriveConstants.holonomicDrive;

    double[] wheelSpeedsLikelyhood = new double[4];
    double[] wheelSpeedsCorrection = new double[4];
    m_autoAlignController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void setProfile(DriveProfiles profile) {
    m_profiles.setCurrentProfile(profile);
  }

  public double calculateMaxAccel(double curSpeed) {
    if (Robot.isReal()) {
      return ModuleConstants.kStartAccel.get() + ModuleConstants.kAccelDropoff.get() * curSpeed;
    } else {
      return ModuleConstants.kStartAccelSim.get() + ModuleConstants.kAccelDropoffSim.get() * curSpeed;
    }
  }


  public void setDriveToPieceChassisSpeeds(ChassisSpeeds speeds) {
    m_driveToPieceSpeeds = speeds;
  }

  public void ffPeriodic() {
    if (m_desChassisSpeeds == null) {
      // System.out.println("Everything is null");
      return;
    }
    double[] m_voltageDrive = new double[4];
    double[] m_voltageTurn = new double[4];
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(m_desChassisSpeeds);
    for (int i = 0; i < moduleStates.length; i++) {
      moduleStates[i] = SwerveModuleState.optimize(moduleStates[i], m_modules[i].getAngle());
    }
    SwerveModuleState[] m_currentModuleStates = getModuleStates();
    SwerveModulePosition[] m_turnStates = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      double desiredSpeed = moduleStates[i].speedMetersPerSecond;
      double desiredAngle = moduleStates[i].angle.getRadians();
      double maxAccel = calculateMaxAccel(m_currentModuleStates[i].speedMetersPerSecond);
      double curAccel = desiredSpeed - m_currentModuleStates[i].speedMetersPerSecond;
      if (curAccel > maxAccel) {
        curAccel = maxAccel;
      }
      double driveFF = m_driveFeedforward.calculate(desiredSpeed, curAccel);
      double drivePID = m_driveFFPIDController.calculate(getModuleStates()[i].speedMetersPerSecond, desiredSpeed);
      double turnPID = m_turnController.calculate(getModuleStates()[i].angle.getRadians(), desiredAngle);
      System.out.println(getModuleStates()[i].angle.getDegrees());
      m_voltageDrive[i] = driveFF;
      m_turnStates[i] = new SwerveModulePosition(0, moduleStates[i].angle);
      m_voltageTurn[i] = turnPID;
      System.out.println("FF Drive" + driveFF + " FF" + turnPID);
    }

    setVoltages(m_voltageDrive, m_voltageTurn);
    // setVoltagesDriveOnly(m_voltageDrive, m_turnStates);
    // double desiredSpeed = swerveModuleState.speedMetersPerSecond * ModuleConstants.kDriveConversionFactor;
    // double desiredAngle = swerveModuleState.angle.getRadians();
    Logger.recordOutput("Drive/DesiredSpeeds",moduleStates);
    // double currentSpeed = getSpeed();
    // double currentAngle = getAngle().getRadians();
    // double driveFF = m_driveFeedforward.calculate(desiredSpeed);
    // double drivePID = m_driveController.calculate(currentSpeed, desiredSpeed);
    // double turnPID = m_turnController.calculate(currentAngle, desiredAngle);
    // System.out
    //     .println("DriveFF: " + driveFF + " DrivePID: " + drivePID + " TurnPID: " + turnPID);
    // setVoltage(driveFF + drivePID, turnPID);
  }

  public void setVoltages(double[] m_voltageDrive, double[] m_voltageTurn) {
    for (int i = 0; i < 4; i++) {
      m_modules[i].setVoltage(m_voltageDrive[i], m_voltageTurn[i]);
    }
  }

  public void setVoltagesDriveOnly(double[] m_voltageDrive, SwerveModulePosition[] modulePositions) {
    for (int i = 0; i < 4; i++) {
      m_modules[i].setVoltageDriveOnly(m_voltageDrive[i], modulePositions[i]);
    }
  }

  public void setDriveTurnOverride(Rotation2d turnOverride){
    this.turnOverride = turnOverride;
  }

  public void setActiveWheel(Integer i) {
    m_activeWheel = i;
  }

  public void defaultPeriodic() {
    if (m_desChassisSpeeds == null) {
      return;
    }
    m_desChassisSpeeds = ChassisSpeeds.discretize(m_desChassisSpeeds, 0.05);
    ModuleLimits currentModuleLimits = RobotState.getInstance().getModuleLimits();
    currentSetpoint =
        setpointGenerator.generateSetpoint(
            currentModuleLimits, currentSetpoint, m_desChassisSpeeds, Constants.loopPeriodSecs);
      ChassisSpeeds finalSpeeds = currentSetpoint.chassisSpeeds();
      SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(m_desChassisSpeeds);
      // SwerveModuleState[] moduleStates = currentSetpoint.moduleStates();
      // Logger.recordOutput("Drive/DesiredSpeedsSetpoint", currentSetpoint.moduleStates() );

  SwerveDriveKinematics.desaturateWheelSpeeds(
        moduleStates,
        m_desChassisSpeeds,
        DriveConstants.kMaxSpeedMetersPerSecond,
        DriveConstants.kMaxSpeedMetersPerSecond,
        DriveConstants.kMaxAngularSpeedRadiansPerSecond);

    
    


    for (int i = 0; i < moduleStates.length; i++) {
      moduleStates[i] = SwerveModuleState.optimize(moduleStates[i], m_modules[i].getAngle());
    }
    SwerveModuleState[] straightStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(m_desChassisSpeeds);
    Logger.recordOutput("Drive/DesiredSpeeds",straightStates);
    Logger.recordOutput("Drive/DesiredSpeedsOptimized",moduleStates);


    // Logger.getInstance().recordOutput("Drive/DesiredModuleStates", moduleStates);
    // Double[] tractionControlStates = calculateTractionLoss(getModuleStates(), getChassisSpeeds(), moduleStates,
    //     m_desChassisSpeeds);

    setModuleStates(moduleStates);

  }

  public void tuningPeriodic() {

  }

  public void setAllowCameraOdometeryConnections(Boolean allow) {
    m_allowCameraOdometeryConnections = allow;
  }

  public void addVisionData(List<TimestampedVisionUpdate> update) {
    // poseEstimator.addVisionData(update);

    // for (TimestampedVisionUpdate a : update) {
    //   m_poseEstimator.addVisionMeasurement(a.pose(), a.timestamp());

    // }
  }

  public void moduleAndAccuracyTesting() {
    m_swerveTester = RobotState.getInstance().getSwerveTester();
    m_swerveTester.runTest();
    if (m_creatingAprilTagFieldLayout) {
      handleCreatingAprilTagFieldLayout();
      return;
    }
    if (!m_singleWheelDriveMode) {
      defaultPeriodic();
    } else {
      handleSingleWheelMode();
    }
  }

  public void handleSingleWheelMode() {
    if (m_moduleState != null) {
      m_modules[m_activeWheel].setDesiredState(m_moduleState);
    } else if (m_moduleVoltageState != null) {
      if (m_moduleVoltageState.m_driveOnly) {
        m_modules[m_activeWheel].setVoltageDriveIgnoreTurn(m_moduleVoltageState.m_driveVoltage);
      } else if (m_moduleVoltageState.m_turnOnly) {
        m_modules[m_activeWheel].setVoltageTurnIgnoreDrive(m_moduleVoltageState.m_driveVoltage);
      } else {
        m_modules[m_activeWheel].setVoltage(m_moduleVoltageState.m_driveVoltage, m_moduleVoltageState.m_turnVoltage);
      }
    }
  }

  public void handleCreatingAprilTagFieldLayout() {
    // check stage of creating april tag field layout
    // stages should be spin to find general relative locations of tags, set smallest tag as origin, drive to nex tag to find approximate distance, then get both tags in view and wait for 1 second of values to average
    // then set the field layout
    if (m_aprilTagFieldCreationStage == AprilTagFieldCreationStage.kInit) {
      resetPose(new Pose2d(2, 2, Rotation2d.fromDegrees(0)));
      m_aprilTagFieldCreationStage = AprilTagFieldCreationStage.kSpinningToFind;
    }
    if (m_aprilTagFieldCreationStage == AprilTagFieldCreationStage.kSpinningToFind) {
      ArrayList<AprilTag> curTags = RobotState.getInstance().getAprilTagsInView();
      for (AprilTag tag : curTags) {
        if (m_tags.indexOf(tag.ID) == -1) {
          m_tags.add(tag.ID);
        }
      }
      m_aprilTagsFoundAsTransforms.addAll(curTags);
      System.out.println(getPose().getRotation().getDegrees());
      System.out.println(m_aprilTagFieldRotation.getDegrees());
      if ((getPose().getRotation().getDegrees() < 20 && getPose().getRotation().getDegrees() > 0)
          || m_aprilTagFieldRotation.getDegrees() > 300 && m_tags.size() > 2) {
        // we have spun around and found all the tags
        m_aprilTagFieldCreationStage = AprilTagFieldCreationStage.kMoveBetweenTags;
        ArrayList<AprilTag> roughLocations = averageAprilTagTransforms(m_aprilTagsFoundAsTransforms);
        System.out.println(roughLocations);
        m_aprilTagsFoundAsTransforms.clear();
        m_aprilTagsFoundAsTransforms.addAll(roughLocations);
        m_aprilTagsFoundAsTransforms.sort((AprilTag a, AprilTag b) -> {
          return a.ID - b.ID;
        });
        System.out.println(m_aprilTagsFoundAsTransforms);
        m_desChassisSpeeds = new ChassisSpeeds(0, 0, 1);
        defaultPeriodic();
      } else {
        m_desChassisSpeeds = new ChassisSpeeds(0, 0, 1);
        defaultPeriodic();
        System.out.println(getPose().getRotation().getDegrees() > 0);
        if (getPose().getRotation().getDegrees() > 0) {
          m_aprilTagFieldRotation = getPose().getRotation();
        } else {
          m_aprilTagFieldRotation = Rotation2d.fromDegrees(360 + getPose().getRotation().getDegrees());
        }
      }
    }
    if (m_aprilTagFieldCreationStage == AprilTagFieldCreationStage.kMoveBetweenTags)

    {
      // if we need to move between tags for accuracy we can do that later
      // for now we will just set the origin to the smallest tag
      convertToNewFieldLayout(m_aprilTagsFoundAsTransforms);
    }

  }

  public void convertToNewFieldLayout(ArrayList<AprilTag> tags) {
    // convert smallest tag to origin
    // convert other tags to be relative to origin
    // set field layout
    // set robot pose to transform based on the distance between the origin and the robot
    Pose3d origin = tags.get(0).pose;
    Transform3d robotToFirstTag = new Transform3d(new Pose3d(getPose()), origin);
    Pose2d newRobotPose = new Pose3d().plus(robotToFirstTag).toPose2d();
    resetPose(newRobotPose);
    ArrayList<AprilTag> newTags = new ArrayList<AprilTag>();
    int maxX = 4;
    int maxY = 4;

    for (int i = 1; i < tags.size(); i++) {
      AprilTag tag = tags.get(i);
      Transform3d tagToOrigin = new Transform3d(tag.pose, origin);
      newTags.add(new AprilTag(tag.ID, new Pose3d().plus(tagToOrigin)));
      if (tag.pose.getTranslation().getX() > maxX) {
        maxX = (int) tag.pose.getTranslation().getX();
      }
      if (tag.pose.getTranslation().getY() > maxY) {
        maxY = (int) tag.pose.getTranslation().getY();
      }
    }

    AprilTagFieldLayout newFieldLayout = new AprilTagFieldLayout(newTags, maxX, maxY);
    RobotState.getInstance().setAprilTagMap(newFieldLayout);
    m_createdAprilTagFieldLayout = true;

  }

  public ArrayList<AprilTag> averageAprilTagTransforms(ArrayList<AprilTag> tags) {
    HashMap<Integer, ArrayList<AprilTag>> tagMap = new HashMap<Integer, ArrayList<AprilTag>>();
    for (AprilTag tag : tags) {
      if (tagMap.containsKey(tag.ID)) {
        tagMap.get(tag.ID).add(tag);
      } else {
        ArrayList<AprilTag> tagList = new ArrayList<AprilTag>();
        tagList.add(tag);
        tagMap.put(tag.ID, tagList);
      }
    }
    ArrayList<AprilTag> averagedTags = new ArrayList<AprilTag>();
    tagMap.forEach((k, v) -> {
      ArrayList<Pose3d> tagPoses = new ArrayList<Pose3d>();
      v.forEach(tag -> {
        tagPoses.add(tag.pose);
      });
      averagedTags.add(new AprilTag(k, average3dPoses(tagPoses)));
    });
    return averagedTags;
  }

  public Pose3d average3dPoses(ArrayList<Pose3d> poses) {
    double x = 0;
    double y = 0;
    double z = 0;
    double xRot = 0;
    double yRot = 0;
    double zRot = 0;
    for (Pose3d pose : poses) {
      x += pose.getTranslation().getX();
      y += pose.getTranslation().getY();
      z += pose.getTranslation().getZ();
      xRot += pose.getRotation().getX();
      yRot += pose.getRotation().getY();
      zRot += pose.getRotation().getZ();
    }
    x /= poses.size();
    y /= poses.size();
    z /= poses.size();
    xRot /= poses.size();
    yRot /= poses.size();
    zRot /= poses.size();
    return new Pose3d(x, y, z, new Rotation3d(xRot, yRot, zRot));
  }

  public void testingPeriodic() {
    switch (m_currentProfileTest) {
      case kWaiting:
        if (ProfilingScheduling.getInstance().checkReadyNextPoint()) {
          setTestProfile(m_currentProfileTest);
        }
      case kFeedForwardAccuracy:
        if (m_profiler == null) {
          String name = "Drive";
          double time = Timer.getFPGATimestamp();
          int subsystemId = DriveConstants.kId;
          int id = 1;
          String[] Units = { "Desired Speed (m/s)", "Actual Speed (m/s)" };
          HardwareProfiler.ProfilingType profilingType = HardwareProfiler.ProfilingType.OTHER;
          int testNumber = 1;
          Double[] testParamters = { 1.0 };
          String[] testParameterNames = { "Tolerance Inches" };
          m_profiler = new HardwareProfiler(name, time, id, subsystemId, Units, profilingType, testNumber,
              testParamters, testParameterNames);

          name = "Power Consumption at Speed";
          Units = new String[] { "Power Used (W)", "Desired Speed (m/s)" };
          id = 3;
          profilingType = HardwareProfiler.ProfilingType.POWER_CONSUMPTION;
          testNumber = 2;
          testParamters = new Double[] {};
          testParameterNames = new String[] {};
          m_profiler2 = new HardwareProfiler(name, time, id, subsystemId, Units, profilingType, testNumber,
              testParamters, testParameterNames);
        }
        double curSetSpeed = DriveConstants.kDriveSpeedTests[m_profileTestIndex];
        ChassisSpeeds s = ChassisSpeeds.fromFieldRelativeSpeeds(curSetSpeed, 0, 0, getPose().getRotation());
        System.out.println(s);
        drive(s);
        if (m_testStartTime == null) {
          m_testStartTime = Timer.getFPGATimestamp();
        }
        if (Timer.getFPGATimestamp() - m_testStartTime > 1.2) {
          m_profiler.addDataPoint(new DataPoint(
              new double[] { getChassisSpeeds().vxMetersPerSecond, curSetSpeed }));
          m_profiler2.addDataPoint(new DataPoint(
              new double[] { m_modules[0].getPowerUsage(), curSetSpeed }));
          m_profileTestIndex++;
          m_testStartTime = null;
          setTestProfile(DriveProfilingSuite.kResetting);
          if (m_profileTestIndex >= DriveConstants.kDriveSpeedTests.length) {
            m_profileTestIndex = 0;
            m_profiler.toJSON();
            m_profiler2.toJSON();
            m_profiler = null;
            m_profiler2 = null;
            m_testStartTime = null;
            // setTestProfile(ElevatorProfilingSuite.kSetpointDeltaAtTime);
          } else {
            // ffPeriodic()
          }
        } else {
          ffPeriodic();
        }
        break;
      case kResetting:
        m_desChassisSpeeds = m_holonomicController.calculate(getPose(),
            new Pose2d(1.81, 6.9, Rotation2d.fromDegrees(0)));
        defaultPeriodic();
        if (m_holonomicController.atReference() && getChassisSpeeds().vxMetersPerSecond < 0.1
            && getChassisSpeeds().vyMetersPerSecond < 0.1 && getChassisSpeeds().omegaRadiansPerSecond < 0.1) {
          setTestProfile(m_lastProfileTest);
        }
        break;
    }
  }

  public Command resetFirmwareCommand() {
    return Commands.none();
    // return run(() -> {
    //   for (SwerveModuleIO module : m_modules) {
    //     module.syncTurningEncoder();
    //     // module.resetEncoders();
    //   }
    // });
  }

  public void resetFirmware() {
    for (SwerveModuleIO module : m_modules) {
      module.syncTurningEncoder();
      module.setUpModuleFirmware();
    }
  }

  public double logMovemnets() {
    double max = 0;
    Logger.recordOutput("Gyro", m_gyro.getAccelX());
    return max;
  }

  public Boolean createAprilTagFieldLayout() {
    if (m_createdAprilTagFieldLayout) {
      m_createdAprilTagFieldLayout = false;
      return true;
    }
    if (!m_creatingAprilTagFieldLayout) {
      m_creatingAprilTagFieldLayout = true;
      m_aprilTagFieldCreationStage = AprilTagFieldCreationStage.kInit;
    }

    return false;

  }

  public void updateSlipData() {
    for (SwerveModuleIO m_io : m_modules) {
      m_io.updateCurrentCalculusSolver();
      m_io.updateWheelSpeedCalculusSolver();
    }
  }

  public void calculateSlipLikelyhood() {
    double[] wheelSpeeds = new double[m_modules.length];
    double[] deltaWheelSpeeds = new double[m_modules.length];
    double[] driveCurrent = new double[m_modules.length];
    double[] deltaDriveCurrent = new double[m_modules.length];
    int i = 0;
    for (SwerveModuleIO m_io : m_modules) {
      wheelSpeeds[i] = m_io.getWheelSpeed();
      deltaWheelSpeeds[i] = m_io.getDeltaWheelSpeed();
      driveCurrent[i] = m_io.getDriveCurrent();
      deltaDriveCurrent[i] = m_io.getDeltaDriveCurrent();
      i++;
    }
    i = 0;

    for (SwerveModuleIO m_io : m_modules) {
      checkWheelSpeedsDifference(wheelSpeeds[i], wheelSpeeds);
      checkDriveCurrentDifference(driveCurrent[i], deltaDriveCurrent[i], wheelSpeeds[i]);
      accelSlipDifferential(m_gyro.getAccel(), deltaWheelSpeeds[i]);
      wheelSlipFactor(wheelSpeeds[i], calculateMaxAccel(wheelSpeeds[i]));

      i++;
    }
    // check for significant difference in drive current
    // check for signifcant difference in wheel speeds

    // wheelSpeedsLikelyhood = 
    // wheelSpeedsCorrection = 
  }

  public double checkWheelSpeedsDifference(double wheelSpeed, double[] wheelSpeeds) {
    double averageWheelSpeeds = 0;
    for (double speed : wheelSpeeds) {
      averageWheelSpeeds += speed;
    }
    averageWheelSpeeds -= wheelSpeed;
    averageWheelSpeeds /= wheelSpeeds.length - 1;
    return Math.abs(wheelSpeed - averageWheelSpeeds) / wheelSpeed;
  }

  public void addVisionMeasurement(Pose2d pose, double timeStamp, Matrix<N3,N1> stds  ){
    // System.out.println("Adding vision measurement");
    // m_poseEstimator.addVisionMeasurement(pose, timeStamp, stds);
  }

  public double checkDriveCurrentDifference(double driveCurrent, double differentialDriveCurrent, double wheelSpeed) {
    return -1 * differentialDriveCurrent * (wheelSpeed / driveCurrent);
  }

  public double accelSlipDifferential(double robotAccel, double wheelAccel) {
    return Math.signum(wheelAccel) * (wheelAccel - robotAccel) / robotAccel;
  }

  public double wheelSlipFactor(double wheelAccel, double maxAccel) {
    return Math.abs(wheelAccel / maxAccel);
  }

  public boolean findSignificantDifference(double[] values, double percentage) {
    double max = 0;
    double min = 0;
    for (double value : values) {
      if (value > max) {
        max = value;
      }
      if (value < min) {
        min = value;
      }
    }
    return (max - min) / max > percentage;
  }
  public void trajectoryFollowingPeriodic(){
    // RobotState.getInstance()
    

  }
  @Override
  public void periodic() {
    // m_DrivePoseEstimator.update(m_gyro.getAngle(), getSwerveModulePositions());
    LoggedTunableNumber.ifChanged(hashCode(), ()->{
      ff = new SimpleMotorFeedforward(ModuleConstants.kffkS.get(),ModuleConstants.kffkV.get(),ModuleConstants.kffkA.get());
  },ModuleConstants.kffkS,ModuleConstants.kffkV);
  LoggedTunableNumber.ifChanged(hashCode(), ()->{
    m_autoAlignController.setP(DriveConstants.AutoAlignP.get());
    m_autoAlignController.setD(DriveConstants.AutoAlignD.get());
  },DriveConstants.AutoAlignP,DriveConstants.AutoAlignD);
  LoggedTunableNumber.ifChanged(hashCode(), ()->{

    m_modules[0].setTurnPID(ModuleConstants.FLkTurningP.get(), ModuleConstants.FLkTurningI.get(), ModuleConstants.FLkTurningD.get());
    m_modules[1].setTurnPID(ModuleConstants.FRkTurningP.get(), ModuleConstants.FRkTurningI.get(), ModuleConstants.FRkTurningD.get());
    m_modules[2].setTurnPID(ModuleConstants.BLkTurningP.get(), ModuleConstants.BLkTurningI.get(), ModuleConstants.BLkTurningD.get());
    m_modules[3].setTurnPID(ModuleConstants.BRkTurningP.get(), ModuleConstants.BRkTurningI.get(), ModuleConstants.BRkTurningD.get());
},ModuleConstants.FLkTurningP,ModuleConstants.FLkTurningI, ModuleConstants.FLkTurningD,ModuleConstants.FRkTurningP,ModuleConstants.FRkTurningI, ModuleConstants.FRkTurningD,ModuleConstants.BLkTurningP,ModuleConstants.BLkTurningI, ModuleConstants.BLkTurningD,ModuleConstants.BRkTurningP,ModuleConstants.BRkTurningI, ModuleConstants.BRkTurningD);
    
odometryLock.lock();
    // Read timestamps from odometry thread and fake sim timestamps
    odometryTimestampInputs.timestamps =
        timestampQueue.stream().mapToDouble(Double::valueOf).toArray();
    if (odometryTimestampInputs.timestamps.length == 0) {
      odometryTimestampInputs.timestamps = new double[] {Timer.getFPGATimestamp()};
    }
    timestampQueue.clear();
    Logger.processInputs("Drive/OdometryTimestamps", odometryTimestampInputs);
    // Read inputs from gyro
    m_gyro.updateInputs(m_gyroInputs);
    Logger.processInputs("Drive/Gyro", m_gyroInputs);
    // Read inputs from modules
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].updateInputs(m_inputs[i]);
      Logger.processInputs("Drive/Module" + i, m_inputs[i]);
    }
    odometryLock.unlock();

    // Calculate the min odometry position updates across all modules

    int minOdometryUpdates =
        IntStream.of(
                odometryTimestampInputs.timestamps.length,
                Arrays.stream(m_moduleNumbers)
                    .map(i -> getModulePositions(i).length)
                    .min()
                    .orElse(0))
            .min()
            .orElse(0);
    
      minOdometryUpdates = Math.min(m_gyroInputs.odometryYawPositions.length, minOdometryUpdates);
    // minOdometryUpdates = Math.min(1,minOdometryUpdates);
    // Pass odometry data to robot state
    SwerveModulePosition[][] positions = new SwerveModulePosition[4][];
    for (int i = 0 ; i < 4;i++){
      positions[i] = getModulePositions(i);


    }

    Logger.recordOutput("Min odo updates", minOdometryUpdates);
    Logger.recordOutput("Drive type", m_profiles.getCurrentProfile().name());
    for (int i = 0; i < minOdometryUpdates; i++) {
      
      int odometryIndex = i;
      Rotation2d yaw =  m_gyroInputs.odometryYawPositions[i];
      // Get all four swerve module positions at that odometry update
      // and store in SwerveDriveWheelPositions object

      SwerveDriveWheelPositions wheelPositions =  
      new SwerveDriveWheelPositions(
            Arrays.stream(m_moduleNumbers)
                .mapToObj(module -> getModulePositions(module)[odometryIndex])
                .toArray(SwerveModulePosition[]::new));
      // getWheelPosititons();
      // RobotState.getInstance()
      //       .addOdometryObservation(
      //           new RobotState.OdometryObservation(
      //               wheelPositions, yaw, odometryTimestampInputs.timestamps[i]));

        // Filtering based on delta wheel positions
        // getWheelPosititons();
      boolean includeMeasurement = true;
      if (lastPositions != null) {
        double dt = odometryTimestampInputs.timestamps[i] - lastTime;
        for (int j = 0; j < m_modules.length; j++) {
          double velocity =
              (wheelPositions.positions[j].distanceMeters
                      - lastPositions.positions[j].distanceMeters)
                  / dt;
          double omega =
              wheelPositions.positions[j].angle.minus(lastPositions.positions[j].angle).getRadians()
                  / dt;
          // Check if delta is too large
          // if (Math.abs(omega) > 28 * 5.0
          //     || Math.abs(velocity) > 5.8 * 5.0) {
          //   includeMeasurement = false;
            
          //   break;
          // }
        }
      }
      Logger.recordOutput("INCLUDING MEASUREMENT", Timer.getFPGATimestamp());
      // If delta isn't too large we can include the measurement.
      if (includeMeasurement) {
        lastPositions = wheelPositions;
        RobotState.getInstance()
            .addOdometryObservation(
                new RobotState.OdometryObservation(
                    wheelPositions, yaw, odometryTimestampInputs.timestamps[i]));
        lastTime = odometryTimestampInputs.timestamps[i];
      }
    }   
    
    ChassisSpeeds robotRelativeVelocity = getChassisSpeeds();
    robotRelativeVelocity.omegaRadiansPerSecond = m_gyroInputs.yawVelocityRadPerSec;
    RobotState.getInstance().addVelocityData(new Twist2d(robotRelativeVelocity.vxMetersPerSecond, robotRelativeVelocity.vyMetersPerSecond, robotRelativeVelocity.omegaRadiansPerSecond));
    // updateSlipData();
    // calculateSlipLikelyhood();
    // Logger.getInstance().processInputs("Gyro", m_gyroInputs);  
    // m_profiles.getPeriodicFunction().run();
    if (m_profiles.getCurrentProfile() == DriveProfiles.kTrajectoryFollowing){
      m_desChassisSpeeds = m_desAutoChassisSpeeds;
      defaultPeriodic();

    }
    else if (m_profiles.getCurrentProfile() == DriveProfiles.kAutoAlign){
      m_desChassisSpeeds = calculateAutoAlignChassisSpeeds();
      defaultPeriodic();
    }
    else if (m_profiles.getCurrentProfile() == DriveProfiles.kAutoPiecePickup){
      
      m_desChassisSpeeds = m_driveToPieceSpeeds;

      defaultPeriodic();
    } else if(m_profiles.getCurrentProfile() == DriveProfiles.kAutoShoot){
      m_desChassisSpeeds = m_autoShootChassisSpeeds;
      m_desChassisSpeeds = calculateAutoAlignChassisSpeeds();
      defaultPeriodic();
    } else if(m_profiles.getCurrentProfile() == DriveProfiles.WHEEL_RADIUS_CHARACTERIZATION){
      m_desChassisSpeeds = new ChassisSpeeds(0, 0, characterizationInput);
      defaultPeriodic();
    } else if(m_profiles.getCurrentProfile() == DriveProfiles.CHARACTERIZATION){
      for(int i = 0; i < 4; i++){
        m_modules[i].runCharacterization(0.0,characterizationInput);
        
      }

    }
    else{
      defaultPeriodic();
    }
    // odometryLock.unlock(); 
    logData();

  }

  public ChassisSpeeds calculateAutoAlignChassisSpeeds(){
    m_desChassisSpeeds.omegaRadiansPerSecond = m_autoAlignController.calculate(getPose().getRotation().getRadians(), turnOverride.getRadians());
    return m_desChassisSpeeds;

  }

  public boolean isTrajectoryComplete(){
      return false;
  }

  public SwerveModulePosition[] getModulePositions(int mod) {
    int minOdometryPositions =
        Math.min(m_inputs[mod].odometryDrivePositionsMeters.length, m_inputs[mod].odometryTurnPositions.length);
    SwerveModulePosition[] positions = new SwerveModulePosition[minOdometryPositions];
    for (int i = 0; i < minOdometryPositions; i++) {
      positions[i] =
          new SwerveModulePosition(
            m_inputs[mod].odometryDrivePositionsMeters[i], m_inputs[mod].odometryTurnPositions[i]);
    }
    return positions;
  }

  public void logData() {
    
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].updateInputs(m_inputs[i]);
      Logger.processInputs("Module" + i, m_inputs[i]);
    }

    m_poseEstimator.update(m_gyro.getAngle(), getSwerveModulePositions());

    Logger.recordOutput("Drive/Pose", getPose());
    Logger.recordOutput("Drive/ModuleStates", getModuleStates());
    Logger.recordOutput("Drive/ModuleAbsoluteStates", getModuleAbsoluteStates());
    Logger.recordOutput("Drive/DesiredStates", getDesiredStates());
    Logger.recordOutput("NorthstarPosition", poseEstimator.getLatestPose());
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      wheelDeltas[i] = new SwerveModulePosition(
          (m_modules[i].getPosition().distanceMeters - lastModulePositionsMeters[i]),
          m_modules[i].getAngle());
      lastModulePositionsMeters[i] = m_modules[i].getPosition().distanceMeters;
    }
    var twist = DriveConstants.kDriveKinematics.toTwist2d(wheelDeltas);
    var gyroYaw = new Rotation2d(m_gyroInputs.yawPositionRad);
    // if (m_gyroInputs.connected) {
      twist = new Twist2d(twist.dx, twist.dy, gyroYaw.minus(lastGyroYaw).getRadians());
    // }
    lastGyroYaw = gyroYaw;

    poseEstimator.addDriveData(Timer.getFPGATimestamp(), twist);
    

    // Log 3D odometry pose
    Pose3d robotPose3d = new Pose3d(getPose());
    robotPose3d = robotPose3d
        .exp(
            new Twist3d(
                0.0,
                0.0,
                m_gyroInputs.pitchPositionRad * DriveConstants.kTrackWidth / 2.0,
                0.0,
                m_gyroInputs.pitchPositionRad,
                0.0))
        .exp(
            new Twist3d(
                0.0,
                0.0,
                m_gyroInputs.rollPositionRad * DriveConstants.kTrackWidth / 2.0,
                m_gyroInputs.rollPositionRad,
                0.0,
                0.0));
    Logger.recordOutput("Odometry/Robot3d", robotPose3d);
    Logger.recordOutput("Odometry/sketchy odo", m_poseEstimator.getEstimatedPosition());
    if (RobotConstants.AScopeLogging) {
      FieldUtil.getDefaultField().setSwerveRobotPose(getPose(), getModuleStates(),
          DriveConstants.kModuleTranslations);
    }
    if (m_lastFPGATimestamp < Timer.getFPGATimestamp()) {
      m_lastFPGATimestamp = Timer.getFPGATimestamp() + 1;
      resetFirmware();
    }

  }

  public SwerveModuleState[] getDesiredStates() {
    if (m_desChassisSpeeds == null) {
      return new SwerveModuleState[] { new SwerveModuleState(0, new Rotation2d(0)) };
    }
    return DriveConstants.kDriveKinematics.toSwerveModuleStates(m_desChassisSpeeds);
  }

  @Override
  public void simulationPeriodic() {
    double gyroDelta = getChassisSpeeds().omegaRadiansPerSecond;
    double ts = Timer.getFPGATimestamp();
    Logger.recordOutput("Drive/Pose", getPose());
    Logger.recordOutput("Drive/ModuleStates", getModuleStates());
    Logger.recordOutput("Drive/ModuleAbsoluteStates", getModuleAbsoluteStates());
    if (RobotConstants.AScopeLogging) {
      FieldUtil.getDefaultField().setSwerveRobotPose(getPose(), getModuleStates(),
          DriveConstants.kModuleTranslations);
    }
    double deltaTime = ts - m_simGyroLastUpdated;

    m_gyro.addAngle(Rotation2d.fromRadians(gyroDelta * deltaTime));
    m_simGyroLastUpdated = ts;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public ChassisSpeeds getChassisSpeedsFieldRelative(){
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getPose().getRotation());
  }

  public void resetOdometry() {
    m_poseEstimator.resetPosition(m_gyro.getAngle(), getSwerveModulePositions(),
        new Pose2d());//1.80, 1.14, new Rotation2d()
    RobotState.getInstance().resetPose(getPose());
    m_hasResetOdometry = true;
  }

  public void resetPose(Pose2d pose) {
    // m_poseEstimator.resetPosition(m_gyro.getAngle(), getSwerveModulePositions(), pose);
    // poseEstimator.resetPose(pose);
    RobotState.getInstance().resetPose(pose);
    m_hasResetOdometry = true;
  }

  public boolean hasResetOdometry() {
    if (m_hasResetOdometry) {
      m_hasResetOdometry = false;
      return true;
    } else {

      return false;
    }
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      positions[i] = m_modules[i].getPosition();
    }
    return positions;
  }

  public SwerveDriveWheelPositions getWheelPosititons(){
    return new SwerveDriveWheelPositions(getSwerveModulePositions());
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] positions = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      positions[i] = m_modules[i].getState();
    }
    return positions;
  }

  public SwerveModuleState setActiveModuleState() {
    return m_modules[m_activeWheel].getState();
  }

  public void setActiveModuleState(SwerveModuleState state) {
    m_moduleState = state;

  }

  public void setActiveModuleVoltageState(SwerveModuleVoltages voltage) {
    m_moduleVoltageState = voltage;
  }

  public void setSingleWheelDriveMode(boolean singleWheelDriveMode) {
    m_singleWheelDriveMode = singleWheelDriveMode;
  }

  public void setChassisSpeedsDirectionOnly(boolean directionOnly) {
    setChassisSpeedsDirectionPIDOnly = directionOnly;
  }

  public Command xBrakeCommand() {
    return run(this::xBrake);
  }

  public Command xBrakeInstantCommand() {
    return runOnce(this::xBrake);
  }

  public void xBrake() {
    SwerveModuleState[] positions = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(m_lockAngles[i])));
    }
  }

  public SwerveModuleState[] getModuleAbsoluteStates() {
    SwerveModuleState[] positions = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      positions[i] = m_modules[i].getAbsoluteState();
    }
    return positions;
  }

  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    // System.out.println("SETTING DESIRED SPEEDS");
    Logger.recordOutput("Drive/DesiredSpeedsInput", moduleStates);

    m_desChassisSpeeds = speeds;
  }

  public void driveAuto(ChassisSpeeds speeds ){
    m_desAutoChassisSpeeds = speeds;
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds);
  }

  public Double[] calculateTractionLoss(SwerveModuleState[] curModules, ChassisSpeeds curSpeeds,
      SwerveModuleState[] desiredModules, ChassisSpeeds desiredSpeeds) {
    Double[] tractionControlStates = new Double[4];
    if (desiredSpeeds.omegaRadiansPerSecond != 0 || curSpeeds.omegaRadiansPerSecond != 0) {
      return tractionControlStates;
    }
    for (int i = 0; i < curModules.length; i++) {

    }
    return tractionControlStates;

  }

  public void setModuleStates(SwerveModuleState[] moduleStates) {
    for (int i = 0; i < moduleStates.length; i++) {
      // m_modules[i].setDesiredState(moduleStates[i]);
      // System.out.println(ff.calculate(moduleStates[i].speedMetersPerSecond/ModuleConstants.kWheelDiameterMeters));
      // System.out.println(moduleStates[i].speedMetersPerSecond/ModuleConstants.kWheelDiameterMeters);
      // System.out.println(ff.calculate((moduleStates[i].speedMetersPerSecond)/(ModuleConstants.kWheelDiameterMeters*Math.PI)));
      if(edu.wpi.first.wpilibj.RobotState.isTeleop()){
      // speed gets multiplied by angular error
      moduleStates[i].speedMetersPerSecond = moduleStates[i].speedMetersPerSecond * Math.cos(moduleStates[i].angle.getRadians()-getModuleStates()[i].angle.getRadians());
      m_modules[i].runDriveVelocitySetpoint(moduleStates[i].speedMetersPerSecond,ff.calculate(moduleStates[i].speedMetersPerSecond,(moduleStates[i].speedMetersPerSecond-getModuleStates()[i].speedMetersPerSecond)*DriveConstants.driveAccel.get()));
      }else{
        m_modules[i].runDriveVelocitySetpoint(moduleStates[i].speedMetersPerSecond,ff.calculate(moduleStates[i].speedMetersPerSecond,(moduleStates[i].speedMetersPerSecond-getModuleStates()[i].speedMetersPerSecond)*DriveConstants.driveAccel.get()));
        // m_modules[i].runDriveVelocitySetpoint(moduleStates[i].speedMetersPerSecond,ff.calculate(moduleStates[i].speedMetersPerSecond,4.3));
      }
      Logger.recordOutput("Desired module turn"+i, Units.radiansToRotations(moduleStates[i].angle.getRotations()));
      m_modules[i].runTurnPositionSetpoint(moduleStates[i].angle.getRadians());
    }
  }

  public Pose2d getPose() {
    // Return the current pose
    // return m_DrivePoseEstimator.getEstimatedPosition();
    return RobotState.getInstance().getEstimatedPose();
  }

  public void brake() {
    // Set chassis speeds to 0
    // setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
  }

  public void addVisionOdometryMeasurement(Pose3d pose, double timestampSeconds) {
    if (m_allowCameraOdometeryConnections) {
      m_poseEstimator.addVisionMeasurement(pose.toPose2d(), timestampSeconds);
    }
  }

  public GyroIO getGyro() {
    return m_gyro;
  }

  public Rotation2d getHeading() {
    return m_gyro.getAngle();
  }

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return m_poseEstimator;
  }

  public Command brakeCommand() {
    return runOnce(this::brake);
  }

  public Command resetCommand(Pose2d resetPose) {
    return runOnce(() -> {
      resetPose(resetPose);
    });
  }

  public void setTestingCommand(DriveProfilingSuite test) {
    m_currentProfileTest = test;
    m_profiles.setCurrentProfile(DriveProfiles.kTesting);
  }

  public Command stopTestingCommand() {
    return runOnce(() -> {
      m_currentProfileTest = null;
      m_profiles.setCurrentProfile(DriveProfiles.kDefault);
    });
  }

  public void setWheelIdleBrake(boolean mode) {
    for (SwerveModuleIO m_io : m_modules) {
      m_io.setBrakeMode(mode);
    }
  }

  // @Override
  public void setTestProfile(Enum<?> profileTest) {
    m_lastProfileTest = m_currentProfileTest;
    m_currentProfileTest = (DriveProfilingSuite) profileTest;
  }

  public static Rotation2d[] getCircleOrientations() {
    return Arrays.stream(DriveConstants.kModuleTranslations)
        .map(translation -> translation.getAngle().plus(new Rotation2d(Math.PI / 2.0)))
        .toArray(Rotation2d[]::new);
  }

  public double[] getWheelRadiusCharacterizationPosition() {
    double[] positions = new double[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = Rotation2d.fromRotations(m_modules[i].getWheelRotations()).getRadians();
    }
    return positions;
  }

  public void runWheelRadiusCharacterization(double omegaSpeed){
    setProfile(DriveProfiles.WHEEL_RADIUS_CHARACTERIZATION);
    characterizationInput = omegaSpeed;
  }


  public Command orientModules(Rotation2d[] orientations) {
    return run(() -> {
        setProfile(DriveProfiles.WHEEL_RADIUS_CHARACTERIZATION);
        characterizationInput = 0;
          for (int i = 0; i < orientations.length; i++) {
            m_modules[i].setTurnPositionSetpoint(orientations[i].getRadians());
          }
        })
        .until(
            () ->
                Arrays.stream(m_modules)
                    .allMatch(
                        module ->
                            EqualsUtil.epsilonEquals(
                                Rotation2d.fromRadians(MathUtil.angleModulus(module.getAngle().getRadians())).getDegrees(),
                                module.getSetpointState().angle.getDegrees(),
                                2.0)))
        .beforeStarting(() -> modulesOrienting = true)
        .finallyDo(() -> modulesOrienting = false)
        .withName("Orient Modules");
  }

  public void runCharacterization(double input) {
    setProfile(DriveProfiles.CHARACTERIZATION);
    characterizationInput = input;
  }

  public void endCharacterization() {
    setProfile(DriveProfiles.kDefault);
  }

  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : m_modules) {
      driveVelocityAverage += module.getState().speedMetersPerSecond;
    }
    return driveVelocityAverage / 4.0;
  }


}
