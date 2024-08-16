// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.utils.SubsystemProfiles;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionObservation;
import frc.robot.utils.LocalADStarAK;

public class Drive extends SubsystemBase {
  private static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
  private static final double TRACK_WIDTH_X = Units.inchesToMeters(25.0);
  private static final double TRACK_WIDTH_Y = Units.inchesToMeters(25.0);
  private static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;

  public enum DriveProfiles {
    kDefault,
    kTrajectoryFollowing,
    kAutoAlign,
    kAutoPiecePickup,
    kAutoShoot,
    kWheelRadiusCharacterization,
    kCharacterization
  }

  private SubsystemProfiles m_profiles;

  private ChassisSpeeds m_desChassisSpeeds = new ChassisSpeeds();
  private ChassisSpeeds m_desAutoChassisSpeeds = new ChassisSpeeds();
  private ChassisSpeeds m_driveToPieceSpeeds = new ChassisSpeeds();

  private Rotation2d m_turnOverride = new Rotation2d();

  private PIDController m_autoAlignController = new PIDController(3.5, 0.0, 0.09);

  private double characterizationInput = 0;

  public boolean modulesOrienting = false;

  private SwerveDriveKinematics kinematics = DriveConstants.kDriveKinematics;
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);
    HashMap<Enum<?>, Runnable> drivePeriodicHash = new HashMap<>();

    drivePeriodicHash.put(DriveProfiles.kDefault, this::defaultPeriodic);
    drivePeriodicHash.put(DriveProfiles.kTrajectoryFollowing, this::trajectoryFollowingPeriodic);
    drivePeriodicHash.put(DriveProfiles.kAutoAlign, this::autoAlignPeriodic);
    drivePeriodicHash.put(DriveProfiles.kAutoPiecePickup, this::autoPiecePickupPeriodic);
    drivePeriodicHash.put(DriveProfiles.kAutoShoot, this::autoShootPeriodic);
    drivePeriodicHash.put(DriveProfiles.kWheelRadiusCharacterization, this::wheelRadiusCharacterizationPeriodic);
    drivePeriodicHash.put(DriveProfiles.kCharacterization, this::characterizationPeriodic);

    Class<? extends Enum<?>> profileEnumClass = DriveProfiles.class;
    Enum<?> defaultProfile = DriveProfiles.kDefault;
    m_profiles = new SubsystemProfiles(profileEnumClass, drivePeriodicHash, defaultProfile);
    // Start threads (no-op for each if no signals have been created)
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));
    
    m_autoAlignController.enableContinuousInput(-Math.PI, Math.PI);
  }
  
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    m_profiles.getPeriodicFunction().run();
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected && RobotBase.isReal()) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);


      Logger.recordOutput("Drive/Profile", m_profiles.getCurrentProfile().name());
    }
  }

  public void setProfile(DriveProfiles profile) {
    m_profiles.setCurrentProfile(profile);
  }

  // Periodic functions
  public void defaultPeriodic(){
    if (m_desChassisSpeeds == null) {
      return;
    }

    runVelocity(m_desChassisSpeeds);

    Logger.recordOutput("Drive/DesiredSpeeds", m_desChassisSpeeds);
  }

  public void trajectoryFollowingPeriodic() {
    if(m_desAutoChassisSpeeds == null){
      defaultPeriodic();
      return;
    }
    
    m_desChassisSpeeds = m_desAutoChassisSpeeds;

    if(RobotState.getInstance().currentTarget != null){
      m_desChassisSpeeds = managePathplannerInconsistency(RobotState.getInstance().currentTarget);
    }
    
    defaultPeriodic();
  }

  public void autoAlignPeriodic() {
    m_desChassisSpeeds = calculateAutoAlignChassisSpeeds();
    defaultPeriodic();
  }

  public void autoPiecePickupPeriodic() {
    m_desChassisSpeeds = m_driveToPieceSpeeds;
    if (m_desChassisSpeeds != null) {
      m_desChassisSpeeds = calculateAutoAlignChassisSpeeds();
    }
    defaultPeriodic();
  }

  public void autoShootPeriodic() {
    m_desChassisSpeeds = calculateAutoAlignChassisSpeeds();
    defaultPeriodic();
  }

  public void wheelRadiusCharacterizationPeriodic() {
    m_desChassisSpeeds = new ChassisSpeeds(0, 0, characterizationInput);
    defaultPeriodic();
  }

  public void characterizationPeriodic() {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(characterizationInput);
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  public void setDesiredSpeed(ChassisSpeeds speeds) {
    m_desChassisSpeeds = speeds;
  }

  public void setDesiredSpeedAuto(ChassisSpeeds speeds) {
    m_desAutoChassisSpeeds = speeds;
  }

  public void setDesiredSpeedDriveToPiece(ChassisSpeeds speeds) {
    m_driveToPieceSpeeds = speeds;
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = DriveConstants.kModuleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPose();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    RobotState.getInstance().resetPose(pose);
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(VisionObservation observation) {
    poseEstimator.addVisionMeasurement(observation);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public ArrayList<Double> getWheelyAmounts(){
    ArrayList<Double> wheelyAmounts = new ArrayList<>();
    wheelyAmounts.add(gyroInputs.pitchVelocityRadPerSec);
    wheelyAmounts.add(gyroInputs.yawVelocityRadPerSec);
    return wheelyAmounts;
  }

  public ChassisSpeeds getChassisSpeedsFieldRelative(){
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getPose().getRotation());
  }

  public Pose2d getPoseTimeAgo() {
    return poseEstimator.getPoseTimeAgo();
  }

  public ChassisSpeeds managePathplannerInconsistency(Pose2d pose){
    if(m_turnOverride != null){
      m_desChassisSpeeds.omegaRadiansPerSecond = m_autoAlignController.calculate(getPose().getRotation().getRadians(), pose.getRotation().getRadians());
    }

    return m_desChassisSpeeds;
  }

  public ChassisSpeeds calculateAutoAlignChassisSpeeds(){
    if(m_turnOverride != null){
      m_desChassisSpeeds.omegaRadiansPerSecond = m_autoAlignController.calculate(getPose().getRotation().getRadians(), m_turnOverride.getRadians());
    }

    return m_desChassisSpeeds;
  }

  public void setDriveTurnOverride(Rotation2d turnOverride){
    System.out.println("Drive turn override set to " + turnOverride.getDegrees() + " degrees");
    m_turnOverride = turnOverride;
  }

  public void runWheelRadiusCharacterization(double omegaSpeed){
    setProfile(DriveProfiles.kWheelRadiusCharacterization);
    characterizationInput = omegaSpeed;
  }

  public double[] getWheelRadiusCharacterizationPosition() {
    double[] positions = new double[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = Rotation2d.fromRotations(modules[i].getWheelRotations()).getRadians();
    }
    return positions;
  }

  

  public ChassisSpeeds getReplayChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public static Rotation2d[] getCircleOrientations() {
    Rotation2d[] orientations = new Rotation2d[4];
    ChassisSpeeds turn = new ChassisSpeeds(0, 0, 1);
    SwerveModuleState[] movements = DriveConstants.kDriveKinematics.toSwerveModuleStates(turn);
    for(int i = 0; i<movements.length; i++){
      orientations[i] = movements[i].angle;
    }
    return orientations;
  }

  public Command orientModules(Rotation2d[] orientations) {
    return runOnce(() -> {
        setProfile(DriveProfiles.kWheelRadiusCharacterization);
        characterizationInput = .1;
          // for (int i = 0; i < orientations.length; i++) {
          //   m_modules[i].runTurnPositionSetpoint(orientations[i].getRadians());
          // }
        }).andThen(Commands.waitSeconds(2))
        .beforeStarting(() -> modulesOrienting = true)
        .finallyDo(() -> modulesOrienting = false)
        .withName("Orient Modules");
  }

  public void runCharacterization(double input){
    setProfile(DriveProfiles.kCharacterization);
    characterizationInput = input;
  }

  public void endCharacterization() {
    setProfile(DriveProfiles.kDefault);
  }

  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }
}
