package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
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
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.hardwareprofiler.DataPoint;
import frc.lib.hardwareprofiler.HardwareProfiler;
import frc.lib.hardwareprofiler.PowerConsumptionHelper;
import frc.lib.hardwareprofiler.ProfiledSubsystem;
import frc.lib.hardwareprofiler.ProfilingScheduling;
import frc.lib.utils.CustomHolmonomicDrive;
import frc.lib.utils.FieldGeomUtil;
import frc.lib.utils.FieldUtil;
import frc.lib.utils.PoseEstimator;
import frc.lib.utils.SubsystemProfiles;
import frc.lib.utils.SwerveModuleVoltages;
import frc.lib.utils.SwerveTester;
import frc.lib.utils.PoseEstimator.TimestampedVisionUpdate;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroInputsAutoLogged;

public class Drive extends ProfiledSubsystem {
  private final SwerveModuleIO[] m_modules;
  private final SwerveModuleInputsAutoLogged[] m_inputs;

  private final SwerveDrivePoseEstimator m_poseEstimator;

  private final GyroIO m_gyro;
  private final GyroInputsAutoLogged m_gyroInputs;

  private final double[] m_lockAngles = new double[] { 45, 315, 45, 315 };
  private boolean m_hasResetOdometry;
  private double m_simGyroLastUpdated;
  private double m_lastFPGATimestamp;

  public ChassisSpeeds m_desChassisSpeeds;
  public SimpleMotorFeedforward m_driveFeedforward;
  public PIDController m_driveController;
  public PIDController m_turnController;
  public PIDController m_driveFFPIDController;
  private double[] lastModulePositionsMeters = new double[] { 0.0, 0.0, 0.0, 0.0 };
  private SubsystemProfiles m_profiles;

  public CustomHolmonomicDrive m_holonomicController;

  Integer m_activeWheel;

  public boolean setChassisSpeedsDirectionPIDOnly = false;
  public boolean m_singleWheelDriveMode = false;

  public enum DriveProfilingSuite {
    kSetpointTime, kSetpointDeltaAtTime, kNone, kResetting, kWaiting, kFeedForwardAccuracy
  }

  public enum DriveProfiles {
    kDefault, kTuning, kTesting, kFFdrive, kFFPIDDrive, kModuleAndAccuracyTesting
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

  public enum AprilTagFieldCreationStage {
    kInit, kSpinningToFind, kMoveToTag, kMoveBetweenTags
  }

  public AprilTagFieldCreationStage m_aprilTagFieldCreationStage = AprilTagFieldCreationStage.kSpinningToFind;

  public ArrayList<AprilTag> m_aprilTagsFoundAsTransforms = new ArrayList<AprilTag>();
  public ArrayList<Integer> m_tags = new ArrayList<Integer>();
  private Rotation2d lastGyroYaw = new Rotation2d();
  Rotation2d m_aprilTagFieldRotation = new Rotation2d(0);

  /** Creates a new Drive. */
  public Drive(GyroIO gyro, Pose2d startPose, SwerveModuleIO... modules) {
    m_modules = modules;
    m_gyro = gyro;
    m_gyroInputs = new GyroInputsAutoLogged();
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
    Class<? extends Enum<?>> profileEnumClass = DriveProfiles.class;
    Enum<?> defaultProfile = DriveProfiles.kFFdrive;
    m_profiles = new SubsystemProfiles(profileEnumClass, drivePeriodicHash, defaultProfile);

    this.m_driveFeedforward = new SimpleMotorFeedforward(ModuleConstants.kDriveKS.get(), ModuleConstants.kDriveKV.get(),
        ModuleConstants.kDriveKA.get());
    this.m_driveController = new PIDController(ModuleConstants.kDriveP.get(), ModuleConstants.kDriveI.get(),
        ModuleConstants.kDriveD.get());
    if (Robot.isReal()) {
      this.m_turnController = new PIDController(ModuleConstants.kTurningP.get(), ModuleConstants.kTurningI.get(),
          ModuleConstants.kTurningD.get());
    } else {
      this.m_turnController = new PIDController(ModuleConstants.kTurningPSim.get(), ModuleConstants.kTurningISim.get(),
          ModuleConstants.kTurningDSim.get());
    }
    this.m_driveFFPIDController = new PIDController(ModuleConstants.kFFDriveP.get(), ModuleConstants.kFFDriveI.get(),
        ModuleConstants.kFFDriveD.get());
    m_holonomicController = DriveConstants.holonomicDrive;

    double[] wheelSpeedsLikelyhood = new double[4];
    double[] wheelSpeedsCorrection = new double[4];
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
      double desiredAngle = moduleStates[i].angle.getDegrees();
      double maxAccel = calculateMaxAccel(m_currentModuleStates[i].speedMetersPerSecond);
      double curAccel = desiredSpeed - m_currentModuleStates[i].speedMetersPerSecond;
      if (curAccel > maxAccel) {
        curAccel = maxAccel;
      }
      double driveFF = m_driveFeedforward.calculate(desiredSpeed, curAccel);
      double drivePID = m_driveFFPIDController.calculate(getModuleStates()[i].speedMetersPerSecond, desiredSpeed);
      double turnPID = m_turnController.calculate(getModuleStates()[i].angle.getDegrees(), desiredAngle);
      m_voltageDrive[i] = driveFF;
      m_turnStates[i] = new SwerveModulePosition(0, moduleStates[i].angle);
      m_voltageTurn[i] = turnPID;
      // System.out.println("FF Drive" + driveFF + " FF" + turnPID);
    }

    // setVoltages(m_voltageDrive, m_voltageTurn);
    setVoltagesDriveOnly(m_voltageDrive, m_turnStates);
    // double desiredSpeed = swerveModuleState.speedMetersPerSecond * ModuleConstants.kDriveConversionFactor;
    // double desiredAngle = swerveModuleState.angle.getRadians();
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

  public void setActiveWheel(Integer i) {
    m_activeWheel = i;
  }

  public void defaultPeriodic() {
    if (m_desChassisSpeeds == null) {
      return;
    }

    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(m_desChassisSpeeds);

    // SwerveDriveKinematics.desaturateWheelSpeeds(
    //     moduleStates,
    //     speeds,
    //     DriveConstants.kMaxModuleSpeedMetersPerSecond,
    //     DriveConstants.kMaxSpeedMetersPerSecond,
    //     DriveConstants.kMaxAngularSpeedRadiansPerSecond);

    for (int i = 0; i < moduleStates.length; i++) {
      moduleStates[i] = SwerveModuleState.optimize(moduleStates[i], m_modules[i].getAngle());
    }
    Logger.getInstance().recordOutput("Drive/DesiredSpeeds",
        moduleStates);

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
    poseEstimator.addVisionData(update);

    for (TimestampedVisionUpdate a : update) {
      m_poseEstimator.addVisionMeasurement(a.pose(), a.timestamp());

    }
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
            // ffPeriodic();
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
    Logger.getInstance().recordOutput("Gyro", m_gyro.getAccelX());
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

  @Override
  public void periodic() {

    m_gyro.updateInputs(m_gyroInputs);
    // updateSlipData();
    // calculateSlipLikelyhood();
    // Logger.getInstance().processInputs("Gyro", m_gyroInputs);  
    m_profiles.getPeriodicFunction().run();
    logData();

  }

  public void logData() {
    
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].updateInputs(m_inputs[i]);
      Logger.getInstance().processInputs("Module" + i, m_inputs[i]);
    }

    m_poseEstimator.update(m_gyro.getAngle(), getSwerveModulePositions());

    Logger.getInstance().recordOutput("Drive/Pose", getPose());
    Logger.getInstance().recordOutput("Drive/ModuleStates", getModuleStates());
    Logger.getInstance().recordOutput("Drive/ModuleAbsoluteStates", getModuleAbsoluteStates());
    Logger.getInstance().recordOutput("Drive/DesiredStates", getDesiredStates());
    Logger.getInstance().recordOutput("NorthstarPosition", poseEstimator.getLatestPose());
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
    Logger.getInstance().recordOutput("Odometry/Robot3d", robotPose3d);
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
    Logger.getInstance().recordOutput("Drive/Pose", getPose());
    Logger.getInstance().recordOutput("Drive/ModuleStates", getModuleStates());
    Logger.getInstance().recordOutput("Drive/ModuleAbsoluteStates", getModuleAbsoluteStates());
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

  public void resetOdometry() {
    m_poseEstimator.resetPosition(m_gyro.getAngle(), getSwerveModulePositions(),
        new Pose2d());//1.80, 1.14, new Rotation2d()
    m_hasResetOdometry = true;
  }

  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_gyro.getAngle(), getSwerveModulePositions(), pose);
    poseEstimator.resetPose(pose);
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
    Logger.getInstance().recordOutput("Drive/DesiredSpeedsInput", moduleStates);

    m_desChassisSpeeds = speeds;
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
      m_modules[i].setDesiredState(moduleStates[i]);
    }
  }

  public Pose2d getPose() {
    // Return the current pose
    return m_poseEstimator.getEstimatedPosition();
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

  @Override
  public void setTestProfile(Enum<?> profileTest) {
    m_lastProfileTest = m_currentProfileTest;
    m_currentProfileTest = (DriveProfilingSuite) profileTest;
  }

}
