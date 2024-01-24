package frc.lib.utils;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.hardwareprofiler.DataPoint;
import frc.lib.hardwareprofiler.HardwareProfiler;
import frc.lib.hardwareprofiler.HardwareProfiler.ProfilingType;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveTester {
  Consumer<Integer> m_setActiveWheel;
  Consumer<ChassisSpeeds> m_setChassisSpeeds;
  Consumer<ChassisSpeeds> m_setChassisSpeedsDirectionOnly;

  Supplier<Rotation2d> m_getGyroAngle;
  Supplier<SwerveModulePosition[]> m_getModulePositions;
  Supplier<SwerveModuleState[]> m_getModuleStates;
  Consumer<SwerveModuleState> m_setModuleState;
  Consumer<SwerveModuleVoltages> m_setModuleVoltageDriveTurn;
  Consumer<SwerveModuleVoltages[]> m_setModulesVoltageDriveTurn;
  Consumer<Boolean> m_setSingleWheelMode;
  Consumer<Boolean> m_setAllowCameraOdometeryConnections;
  Supplier<ChassisSpeeds> m_getChassisSpeeds;
  Supplier<Boolean> m_createAprilTagLayout;
  Supplier<Pose2d> m_getPose;
  Consumer<Pose2d> m_setPose;
  Supplier<ArrayList<Pose2d>> m_getCurrentVisionPose;
  Supplier<AprilTagFieldLayout> m_getAprilTagLayout;
  SwerveTesterProfiles m_currentTest;
  SwerveDriveKinematics m_kinematics;
  HardwareProfiler m_profiler;
  double m_trackCircleDiameter;

  ArrayList<Pose2d> currentPoses = new ArrayList<Pose2d>();

  double m_lastRelevantTime;
  int m_currentTestNum;
  Rotation2d m_lastGyroAngle;
  boolean m_testRunning = false;

  Pose2d m_testStartPose;

  public enum SwerveTesterProfiles {
    kTimeToSpeed, kOdometryTest, kWheelSlip, kForwardTurnTest, kModuleTimeToSpeed, kVoltageAcceleration,
    kModuleToModuleSpeedDifferentialAtSameSpeed, kNone
  }

  public boolean m_isTesting = false;
  public Integer m_wheelNumber = 0;

  public SwerveTester(
      Consumer<Integer> setActiveWheel, //
      Consumer<ChassisSpeeds> setChassisSpeeds, //
      Supplier<Rotation2d> getGyroAngle, //
      Supplier<SwerveModulePosition[]> getModulePositions, //
      Supplier<SwerveModuleState[]> getModuleStates, //
      Consumer<SwerveModuleState> setModuleState,
      Consumer<SwerveModuleVoltages> setModuleVoltageState,
      SwerveDriveKinematics kinematics,
      Consumer<Boolean> setChassisSpeedsDirectionOnly,
      Supplier<ChassisSpeeds> getChassisSpeeds,
      Consumer<Boolean> setBrakeMode,
      Consumer<Boolean> setSingleWheelMode,
      Consumer<Boolean> setAllowCameraOdometeryConnections,
      Supplier<Boolean> createAprilTagLayout,
      Supplier<Pose2d> getPose,
      Consumer<Pose2d> setPose,
      Supplier<ArrayList<Pose2d>> getCurrentVisionPose,
      Supplier<AprilTagFieldLayout> getAprilTagLayout,
      double trackCircleDiameter) {
    m_setActiveWheel = setActiveWheel;
    m_setChassisSpeeds = setChassisSpeeds;
    m_getGyroAngle = getGyroAngle;
    m_getModulePositions = getModulePositions;
    m_getModuleStates = getModuleStates;
    m_setModuleState = setModuleState;
    m_kinematics = kinematics;
    m_setModuleVoltageDriveTurn = setModuleVoltageState;
    m_getChassisSpeeds = getChassisSpeeds;
    m_setSingleWheelMode = setSingleWheelMode;
    m_createAprilTagLayout = createAprilTagLayout;
    m_setAllowCameraOdometeryConnections = setAllowCameraOdometeryConnections;
    m_getPose = getPose;
    m_setPose = setPose;
    m_getCurrentVisionPose = getCurrentVisionPose;
    m_getAprilTagLayout = getAprilTagLayout;
    m_trackCircleDiameter = trackCircleDiameter;
  }

  public void setCurrentSwerveTest(SwerveTesterProfiles test) {
    m_currentTest = test;
  }

  public void startTesting() {
    m_isTesting = true;
  }

  public void stopTesting() {
    m_isTesting = false;
  }

  public void resetTestingValues() {
    m_lastRelevantTime = -1;
    m_currentTestNum = -1;
    m_profiler = null;
  }

  public boolean runIndividualWheelModuleTimeToSpeed() {
    if (m_profiler == null) {
      resetTestingValues();
      m_profiler = new HardwareProfiler("Module Time To Speed", 0.0, 2, 10,
          new String[] { "Seconds", "Meters/Seconds" },
          new ArrayList<DataPoint>(),
          ProfilingType.TIME_TO_SETPOINT, m_wheelNumber, new Double[] { 0.1 }, new String[] { "Tolerance Velocity" });
    }
    if (!m_testRunning) {
      if (isMovingAndStop()) {
        return false;
      } else {
        m_currentTestNum += 1;
        m_lastRelevantTime = Timer.getFPGATimestamp();
        m_lastGyroAngle = m_getGyroAngle.get();
        m_testRunning = true;
        System.out.println("Increasing ");
        if (m_currentTestNum >= Constants.DriveConstants.kDriveSpeedTests.length) {
          m_isTesting = false;
          m_profiler.toJSON();
          resetTestingValues();
          return true;
        }
        return false;
      }
    }
    if (m_isTesting) {
      double speed = Constants.DriveConstants.kDriveSpeedTests[m_currentTestNum];
      m_setSingleWheelMode.accept(true);
      // m_setModuleVoltageDriveTurn
      //     .accept(SwerveModuleVoltages.setDriveOnly());

      // double time = Timer.getFPGATimestamp() - m_lastRelevantTime;
      double actualSpeed = Math.abs(m_getModuleStates.get()[m_wheelNumber].speedMetersPerSecond);
      System.out.println(actualSpeed);
      System.out.println(Constants.DriveConstants.kDriveSpeedTests[m_currentTestNum]);
      System.out.println(speed);
      System.out.println(Math.abs(Constants.DriveConstants.kDriveSpeedTests[m_currentTestNum] - actualSpeed));

      if (Math.abs(Constants.DriveConstants.kDriveSpeedTests[m_currentTestNum] - actualSpeed) < 0.1) {
        m_profiler.addDataPoint(new DataPoint(new double[] { Timer.getFPGATimestamp() - m_lastRelevantTime, speed }));
        m_lastRelevantTime = -1;
        m_testRunning = false;
      }
      return false;
    }
    return false;
  }

  public void moduleTimeToSpeed() {
    if (m_wheelNumber == 4) {
      m_wheelNumber = 0;
      setCurrentSwerveTest(SwerveTesterProfiles.kNone);
      return;
    }
    if (runIndividualWheelModuleTimeToSpeed()) {
      m_wheelNumber += 1;
    }

  }

  public boolean isMovingAndStop() {
    if ((m_getChassisSpeeds.get().vxMetersPerSecond > 0.1
        || checkDirectionEquivilence(m_getModuleStates.get(), new ChassisSpeeds(0, 0, 1)) > 1
        || m_getChassisSpeeds.get().omegaRadiansPerSecond > 1)) {
      m_setSingleWheelMode.accept(false);
      m_setChassisSpeeds.accept(new ChassisSpeeds(0, 0, 0.0001));
      return true;
    } else {
      return false;
    }
  }

  public void odometryTestStraight() {
    m_setAllowCameraOdometeryConnections.accept(false);
    if (m_profiler == null) {
      resetTestingValues();
      if (!m_createAprilTagLayout.get()) {
        return;
      }
      m_profiler = new HardwareProfiler("Error in driving to points without vision", 0.0, 3, 10,
          new String[] { "Error Inches", "Meters/Seconds" },
          new ArrayList<DataPoint>(),
          ProfilingType.TIME_TO_SETPOINT, m_wheelNumber, new Double[] { 0.1 }, new String[] { "Tolerance Velocity" });
    }
    if (!m_testRunning) {
      if (slowResetInFrontOfFirstAprilTag()) {
        return;
      } else if (averageCameraPoseForASecond()) {
        return;
      } else {
        m_currentTestNum += 1;
        // m_lastRelevantTime = Timer.getFPGATimestamp();
        m_testRunning = true;
        if (m_currentTestNum >= Constants.DriveConstants.kDriveSpeedTests.length) {
          m_isTesting = false;
          m_profiler.toJSON();
          resetTestingValues();
          return;
        }
        return;
      }
    }

    if (m_testRunning) {
      double speed = Constants.DriveConstants.kDriveSpeedTests[m_currentTestNum];
      // find the desired pose (the pose of the 2nd lowest april tag)
      System.out.println(m_getAprilTagLayout.get().getTags());
      // Logger.getInstance().recordOutput("Tag 1", m_getAprilTagLayout.get().getTagPose(1).get());
      // Logger.getInstance().recordOutput("Tag 2", m_getAprilTagLayout.get().getTagPose(2).get());
      // Logger.getInstance().recordOutput("Tag 3", m_getAprilTagLayout.get().getTagPose(3).get());
      Pose2d desiredPose = m_getAprilTagLayout.get().getTagPose(2).get().toPose2d();
      desiredPose = desiredPose.plus(new Transform2d(new Translation2d(1, 0), Rotation2d.fromDegrees(0)));
      // find the current pose
      Pose2d currentPose = m_getCurrentVisionPose.get().get(0);
      // find the error
      Transform2d error = desiredPose.minus(currentPose);

      // // find the speed to set
      ChassisSpeeds speeds = getChassisSpeedsInDirection(error, speed, currentPose, desiredPose);
      m_setChassisSpeeds.accept(speeds);
      if (DriveConstants.holonomicDrive.atReference(currentPose, desiredPose,
          new Pose2d(.2, .2, Rotation2d.fromRadians(.3)))) {
        if (justAverageCameraPoseForASecondWithoutResetPose()) {
          Pose2d visionPose = averagePoses(currentPoses);
          Pose2d currentOdometryPose = m_getPose.get();
          Transform2d errorPose = visionPose.minus(currentOdometryPose);
          double errorInches = errorPose.getTranslation().getNorm() * 39.3701;
          m_profiler.addDataPoint(new DataPoint(new double[] { errorInches, speed }));
          m_setPose.accept(visionPose);
          m_lastRelevantTime = -1;
          m_testRunning = false;
        }

      }
    }

  }

  public ChassisSpeeds getChassisSpeedsInDirection(Transform2d error, Double maxSpeed, Pose2d start, Pose2d end) {
    double distanceToTarget = error.getTranslation().getNorm();

    if (DriveConstants.holonomicDrive.atReference(start, end, new Pose2d(.2, .2, Rotation2d.fromRadians(.3)))) {
      return new ChassisSpeeds(0, 0, 0);
    }
    ChassisSpeeds speeds = Constants.DriveConstants.holonomicDrive.calculate(start, end);
    double speed = Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));
    if (speed > maxSpeed) {
      speeds = new ChassisSpeeds(speeds.vxMetersPerSecond / speed * maxSpeed,
          speeds.vyMetersPerSecond / speed * maxSpeed, speeds.omegaRadiansPerSecond);
    }
    return speeds;

  }

  public boolean averageCameraPoseForASecond() {
    if (m_lastRelevantTime == -1) {
      m_lastRelevantTime = Timer.getFPGATimestamp();
      currentPoses.clear();
    }
    if (Timer.getFPGATimestamp() - m_lastRelevantTime > 1) {
      m_lastRelevantTime = -1;
      m_setPose.accept(averagePoses(currentPoses));
      return true;
    }
    ArrayList<Pose2d> curPose = m_getCurrentVisionPose.get();
    if (curPose != null) {
      currentPoses.addAll(curPose);
    }
    return false;
  }

  public boolean justAverageCameraPoseForASecondWithoutResetPose() {
    if (m_lastRelevantTime == -1) {
      m_lastRelevantTime = Timer.getFPGATimestamp();
      currentPoses.clear();
    }
    if (Timer.getFPGATimestamp() - m_lastRelevantTime > 1) {
      m_lastRelevantTime = -1;
      return true;
    }
    ArrayList<Pose2d> curPose = m_getCurrentVisionPose.get();
    if (curPose != null) {
      currentPoses.addAll(curPose);
    }
    return false;
  }

  public Pose2d averagePoses(ArrayList<Pose2d> poses) {
    double x = 0;
    double y = 0;
    double angle = 0;
    for (Pose2d pose : poses) {
      x += pose.getTranslation().getX();
      y += pose.getTranslation().getY();
      angle += pose.getRotation().getRadians();
    }
    return new Pose2d(x / poses.size(), y / poses.size(), new Rotation2d(angle / poses.size()));
  }

  public boolean slowResetInFrontOfFirstAprilTag() {
    Pose2d currentPose = m_getPose.get();
    Pose2d desiredPose = new Pose2d(1, 0, new Rotation2d(0));
    Transform2d transform = currentPose.minus(desiredPose);
    if (transform.getTranslation().getX() < 0.1 && transform.getTranslation().getY() < 0.1
        && transform.getRotation().getRadians() < 0.1) {
      m_setChassisSpeeds.accept(new ChassisSpeeds(0, 0, 0));
      return true;
    } else {
      // drive towards desired pose at max at .5 meters per second in the x direction and .5 meters per second in the y direction
      m_setChassisSpeeds.accept(new ChassisSpeeds(Math.min(transform.getX(), 0.5), Math.min(transform.getY(), 0.5),
          Math.min(transform.getRotation().getRadians(), 0.2)));
      return false;

    }

  }

  public void timeToSpeed() {
    if (m_profiler == null) {
      resetTestingValues();
      m_profiler = new HardwareProfiler("Time To Speed", 0.0, 1, 10, new String[] { "Seconds", "Meters/Seconds" },
          new ArrayList<DataPoint>(),
          ProfilingType.TIME_TO_SETPOINT, 1, new Double[] { 0.1 }, new String[] { "Tolerance Velocity" });
      System.out.println("resetting All");
    }
    if (!m_testRunning) {
      if (isMovingAndStop()) {
        return;
      } else {
        m_currentTestNum += 1;
        m_lastRelevantTime = Timer.getFPGATimestamp();
        m_lastGyroAngle = m_getGyroAngle.get();
        m_testRunning = true;
        System.out.println("Increasing ");
        if (m_currentTestNum >= Constants.DriveConstants.kDriveSpeedTests.length) {
          m_isTesting = false;
          setCurrentSwerveTest(SwerveTesterProfiles.kNone);
          m_profiler.toJSON();
          resetTestingValues();
          return;
        }
        return;
      }
    }
    if (m_isTesting) {
      double speed = convertSpeedToAngularVelocity(Constants.DriveConstants.kDriveSpeedTests[m_currentTestNum], 1);
      m_setChassisSpeeds.accept(new ChassisSpeeds(0, 0, speed));

      // double time = Timer.getFPGATimestamp() - m_lastRelevantTime;
      double actualSpeed = Math.abs(m_getModuleStates.get()[0].speedMetersPerSecond);
      System.out.println(actualSpeed);
      System.out.println(Constants.DriveConstants.kDriveSpeedTests[m_currentTestNum]);
      System.out.println(speed);
      System.out.println(Math.abs(Constants.DriveConstants.kDriveSpeedTests[m_currentTestNum] - actualSpeed));

      if (Math.abs(Constants.DriveConstants.kDriveSpeedTests[m_currentTestNum] - actualSpeed) < 0.1) {
        m_profiler.addDataPoint(new DataPoint(new double[] { Timer.getFPGATimestamp() - m_lastRelevantTime, speed }));
        m_lastRelevantTime = -1;
        m_testRunning = false;
      }
    }

  }

  public double checkDirectionEquivilence(SwerveModuleState[] m_states, ChassisSpeeds chassisSpeeds) {
    double maxError = 0;

    SwerveModuleState[] m_cor = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    for (int i = 0; i < m_states.length; i++) {
      double error = Math.abs(m_states[i].angle.getDegrees() - m_cor[i].angle.getDegrees());
      if (error > 90) {
        error = 180 - error;
      }
      if (error > maxError) {
        maxError = error;
      }
    }
    // System.out.println(maxError);
    return maxError;

  }

  public double convertAngularVelocityToWheelSpeed(Rotation2d angle, double timeDeltaSeconds) {
    double angularVelocity = angle.getRadians() / timeDeltaSeconds; // ASSUMES ROBOT IS SQUARE
    double wheelSpeed = angularVelocity * m_trackCircleDiameter / 2.0;
    return wheelSpeed;
  }

  public double convertSpeedToAngularVelocity(double wheelSpeed, double timeDeltaSeconds) {
    double angularVelocity = wheelSpeed / (m_trackCircleDiameter / 2.0);
    double angle = angularVelocity * timeDeltaSeconds * 2 * Math.PI;
    return angle;
  }

  public void runTest() {
    if (m_isTesting) {
      switch (m_currentTest) {
        case kTimeToSpeed:
          timeToSpeed();
          break;
        case kOdometryTest:
          odometryTestStraight();
          break;
        // case kWheelSlip:
        //   wheelSlip();
        //   break;
        // case kForwardTurnTest:
        //   forwardTurnTest();
        //   break;
        case kModuleTimeToSpeed:
          moduleTimeToSpeed();
          break;
        // case kVoltageAcceleration:
        //   voltageAcceleration();
        //   break;
        // case kModuleToModuleSpeedDifferentialAtSameSpeed:
        //   moduleToModuleSpeedDifferentialAtSameSpeed(); THIS MUST MAKE THE ROBOT GO FORWARD TO BE ACCURATE
        //   break;
        default:
          break;
      }
    }
  }

}
