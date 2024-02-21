// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.autonPlanning.ChoreoManager;
import frc.lib.utils.NetworkTablesTEBInterfacer;
import frc.lib.utils.VirtualSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Ports;
import frc.robot.commands.autonomous.AutoFactory;
import frc.robot.commands.drive.TeleopControllerNoAugmentation;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsDualFlightStick;
import frc.robot.oi.DriverControlsXboxController;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIOKraken;
import frc.robot.subsystems.drive.SwerveModuleIOMK4Talon;
import frc.robot.subsystems.drive.SwerveModuleIOMK4iSparkMax;
import frc.robot.subsystems.drive.SwerveModuleIOMK4iSparkMaxOld;
import frc.robot.subsystems.drive.SwerveModuleIOSim;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon;
import frc.robot.subsystems.drive.gyro.GyroIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.northstarAprilTagVision.AprilTagVisionIONorthstar;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.pivot.PivotIO;
import frc.robot.subsystems.shooter.pivot.PivotIOSim;
import frc.robot.utils.ShooterMath;

public class RobotContainer {

  RobotState m_robotState;
  Drive m_drive;
  Intake m_intake;
  AutoFactory m_autoFactory;
  Shooter m_shooter;
  frc.robot.subsystems.northstarAprilTagVision.AprilTagVision m_aprilTagVision;
  NetworkTablesTEBInterfacer m_TebInterfacer;
  Climb m_climb;

  
  DriverControls m_driverControls;


  private LoggedDashboardChooser<Command> m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser");

  public RobotContainer() {
    configureControllers();
    configureBindings();
    configureSubsystems();
    configureCommands();
    configureAutonomous();
  }

  public void configureControllers(){
    m_driverControls = new DriverControlsXboxController(4);
    
    
  }

  public void configureCommands(){
    m_drive.setDefaultCommand(new TeleopControllerNoAugmentation(m_drive,()->m_driverControls.getDriveForward(),()->m_driverControls.getDriveLeft() , ()-> m_driverControls.getDriveRotation(), DriveConstants.controllerDeadzone));
    m_driverControls.resetFieldCentric().onTrue(Commands.runOnce(()->m_drive.resetPose(new Pose2d())));
    m_driverControls.setShooter45().onTrue(Commands.runOnce(()->{
      System.out.println("Setting shooter to 45");
      m_shooter.setPivotAngle(Rotation2d.fromDegrees(45));}));
      m_driverControls.setClimberServoClose().onTrue(Commands.runOnce(()->{
        System.out.println("Setting climber servo to close");
        m_climb.setServoPosition(0);}));

      m_driverControls.setClimberServoOpen().onTrue(Commands.runOnce(()->{
        System.out.println("Setting climber servo to open");
        m_climb.setServoPosition(0.25);
      }));

        // m_climb.deltaServoPosition(.01);}));
  

      m_driverControls.setClimberServoMove().onTrue(Commands.runOnce(()->{
        m_climb.deltaServoPosition(-0.01);
      }));
  }

  public void planChoreo(){
    m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
    

    // Add basic autonomous commands
    m_autoChooser.addDefaultOption("Do Nothing", Commands.none());
    List<String> paths = ChoreoManager.getExistingPaths();
    for (String path : paths) {
      m_autoChooser.addOption(path, m_autoFactory.getAutoCommand(path));
    }
  }

  public void configureAutonomous(){
    m_autoFactory = new AutoFactory(m_drive, null);
    planChoreo();
  }

  public void configureSubsystems(){
    if (Robot.isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    m_climb = new Climb();
    m_intake = new Intake(new frc.robot.subsystems.intake.pivot.PivotIOSim(), new frc.robot.subsystems.intake.rollers.RollerIOKraken(Ports.intakeMotorPort,m_driverControls::intakeNote));
    // Logger.recordOutput("kShooterBackLeft", new Pose3d(FieldConstants.kShooterBackLeft, new Rotation3d(0, 0, 0)));
    // Logger.recordOutput("kShooterBackRight", new Pose3d(FieldConstants.kShooterBackRight, new Rotation3d(0, 0, 0)));
    // Logger.recordOutput("kShooterFrontLeft", new Pose3d(FieldConstants.kShooterFrontLeft, new Rotation3d(0, 0, 0)));
    // Logger.recordOutput("kShooterFrontRight", new Pose3d(FieldConstants.kShooterFrontRight, new Rotation3d(0, 0, 0)));


    // Instantiate our RobotContainer. This will perform all our button bindings,

    

    
    
    m_shooter = new Shooter(new PivotIOSim(), new FlywheelIOSim());
    if (Robot.isSimulation()) {
      m_drive = new Drive(new GyroIOPigeon(22, new Rotation2d()), new Pose2d(),
          new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim());
      // new SwerveModuleIOMK4Talon(1,2,3),
      //     new SwerveModuleIOMK4Talon(4,5,6),
      //     new SwerveModuleIOMK4Talon(7,8,9),
      //     new SwerveModuleIOMK4Talon(10,11,12));
      m_aprilTagVision = new frc.robot.subsystems.northstarAprilTagVision.AprilTagVision(new AprilTagVisionIONorthstar("northstar_0"));
    }
    else{
      // m_shooter = new Shooter(new PivotIO());
    //   SwerveModuleIO[] m_swerveModuleIOs = {
    //     new SwerveModuleIOMK4iSparkMax(Constants.Ports.leftFrontDrivingMotorPort, Ports.leftFrontTurningMotorPort,
    //         Ports.leftFrontCanCoderPort),
    //     new SwerveModuleIOMK4iSparkMax(Constants.Ports.rightFrontDriveMotorPort, Ports.rightFrontTurningMotorPort,
    //         Ports.rightFrontCanCoderPort),
    //     new SwerveModuleIOMK4iSparkMax(Constants.Ports.leftRearDriveMotorPort, Ports.leftRearTurningMotorPort,
    //         Ports.leftRearCanCoderPort),
    //     new SwerveModuleIOMK4iSparkMax(Constants.Ports.rightRearDriveMotorPort, Ports.rightRearTurningMotorPort,
    //         Ports.rightRearCanCoderPort) };
    // m_drive = new Drive(new GyroIOPigeon(Constants.Ports.pigeonPort, Constants.DriveConstants.pitchAngle),
    //     Constants.DriveConstants.startPose,
    //     m_swerveModuleIOs);

        SwerveModuleIO[] m_SwerveModuleIOs = {
          new SwerveModuleIOKraken(1,2,3,false),
          // new SwerveModuleIOMK4Talon(1,2,3),
          new SwerveModuleIOKraken(4,5, 6, false),
          // new SwerveModuleIOSim(),
          new SwerveModuleIOKraken(7, 8, 9, false),
          // new SwerveModuleIOMK4Talon(4,5,6),
          new SwerveModuleIOKraken(10, 11, 12, false),
          // new SwerveModuleIOSim(),
          // new SwerveModuleIOMK4Talon(7,8,9),
          // new SwerveModuleIOSim(),
          // new SwerveModuleIOMK4Talon(10,11,12),
        };
        m_drive = new Drive(new GyroIOPigeon(22,new Rotation2d()),new Pose2d(),m_SwerveModuleIOs);
      m_aprilTagVision = new frc.robot.subsystems.northstarAprilTagVision.AprilTagVision(new AprilTagVisionIONorthstar("northstar_0"));
    }
    
    m_robotState = RobotState.startInstance(m_drive, null, null,m_shooter, null, m_aprilTagVision, m_intake);
    m_TebInterfacer = new NetworkTablesTEBInterfacer("teb");
  }

  public void updateRobotState(){
    VirtualSubsystem.periodicAll(); 
    m_robotState.updateRobotState();
    m_robotState.calculateShooterAngle();
    m_TebInterfacer.update();
  }

  public void onDisabled(){
    m_drive.setWheelIdleBrake(true);
  }

  public void disabledPeriodic(){
    
  }

  public void onEnabled(){
    m_robotState.updateTestScheduler();
    
  }
  private void configureBindings() {
    Transform3d cameraToObject =  new Transform3d(new Translation3d(2,3,0), new Rotation3d());
    Transform3d robotToCamera = new Transform3d(new Translation3d(0,0,1), new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(30),Units.degreesToRadians(0)));
    // advantagekit log
    Pose3d objectPose = new Pose3d().plus(robotToCamera).plus(cameraToObject);
    Logger.recordOutput("objectPose", objectPose); 
    Logger.recordOutput("Camera Pose", new Pose3d().plus(robotToCamera));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.get();
    // return Commands.print("No autonomous command configured");
  }
}
