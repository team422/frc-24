// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.autonPlanning.PathPlannerUtil;
import frc.lib.utils.NetworkTablesTEBInterfacer;
import frc.lib.utils.VirtualSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Ports;
import frc.robot.RobotState.RobotCurrentAction;
import frc.robot.commands.autonomous.AutoFactory;
import frc.robot.commands.drive.TeleopControllerNoAugmentation;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsXboxController;
import frc.robot.oi.ManualController;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOTalon;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveProfiles;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIOKraken;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.northstarAprilTagVision.AprilTagVisionIONorthstar;
import frc.robot.subsystems.objectVision.ObjectDetectionCam;
import frc.robot.subsystems.objectVision.ObjectDetectionIODepth;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterIsIntaking;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOKraken;
import frc.robot.subsystems.shooter.pivot.PivotIOFalcon;
import frc.robot.utils.AllianceFlipUtil;

public class RobotContainer {

  RobotState m_robotState;
  Drive m_drive;
  Intake m_intake;
  AutoFactory m_autoFactory;
  Shooter m_shooter;
  frc.robot.subsystems.northstarAprilTagVision.AprilTagVision m_aprilTagVision;
  NetworkTablesTEBInterfacer m_TebInterfacer;
  Climb m_climb;
  Indexer m_indexer;
  
  DriverControls m_driverControls;
  ManualController m_testingController;

  ObjectDetectionCam m_objectDetectionCams;

  Command autoDriveCommand;


  private LoggedDashboardChooser<Command> m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser");

  public RobotContainer() {
    configureControllers();
    configureBindings();
    configureSubsystems();
    configureCommands();
  }

  public void configureControllers(){
    m_driverControls = new DriverControlsXboxController(4);
    m_testingController = new ManualController(3);
    
  }

  public void configureCommands(){
    // m_drive.setDefaultCommand();
    
    m_driverControls.resetFieldCentric().onTrue(Commands.runOnce(()->m_drive.resetPose(new Pose2d(m_robotState.getEstimatedPose().getTranslation(),AllianceFlipUtil.apply(Rotation2d.fromDegrees(180))))));
      m_driverControls.setClimberServoClose().onTrue(Commands.runOnce(()->{
        // System.out.println("Setting climber servo to close");
        m_climb.setServoPosition(0);}));

      m_driverControls.setClimberServoOpen().onTrue(Commands.runOnce(()->{
        // System.out.println("Setting climber servo to open");
        m_climb.setServoPosition(0.25);
      }));

      m_driverControls.finalShoot().onTrue(Commands.runOnce(()->{
        System.out.println("NOW SHOOT");
        m_indexer.setState(IndexerState.SHOOTING);
      }));
      // .onFalse(Commands.runOnce(()->{
      //   m_robotState.setRobotCurrentAction(RobotCurrentAction.kStow);
      // }));


      m_driverControls.runTune().onTrue(Commands.runOnce(()->{
        // m_robotState.setRobotCurrentAction(RobotCurrentAction.kTune);
        m_indexer.setState(IndexerState.INDEXING);
        m_intake.setIntakeSpeed(0);
      }));


      m_testingController.amp().onTrue(Commands.runOnce(()->{
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kAmpShoot);
      }));

      m_driverControls.hockeyPuck().whileTrue(Commands.runOnce(()->{
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kHockeyPuck);
      })).onFalse(Commands.runOnce(()->{
        Logger.recordOutput("stow Trigger 1", Timer.getFPGATimestamp());
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kStow);
        m_robotState.setDriveType(DriveProfiles.kDefault);
      }));
      



      m_testingController.cimberLockToggle().onTrue(Commands.runOnce(()->{
        m_climb.toggleServo();
      }));

      new Trigger(()->{if(Math.abs(m_testingController.climberStick()) > 0.2) {return true;} else {return false;}}).whileTrue(Commands.run(()->m_climb.setSpeed(m_testingController.climberStick()))).onFalse(Commands.runOnce(()->m_climb.setSpeed(0)));
      // new Trigger(()->{if(Math.abs(m_testingController.climberStick()) > 0.2){return true;}else{return false;}} ) .whileTrue(Commands.runOnce(()->{
      //   m_climb.setSpeed(m_testingController.climberStick());
      // })).whileFalse(Commands.runOnce(()->m_climb.setSpeed(0)));

      

      // new Trigger(()->{if(Math.abs(m_testingController.getFeederStick()) > 0.2){return true;}else{return false;}} ).onTrue(Commands.runOnce(()->{
      //   // m_indexer.setManualSpeed(m_testingController.getFeederStick());
      //   System.out.println("STARTING INDEXING");
      //   m_indexer.setState(IndexerState.INDEXING);
      // }));

      // new Trigger(()->{if(Math.abs(m_testingController.getShooterStick()) > 0.2){return true;}else{return false;}} ).onTrue(Commands.run(()->{
      //   m_shooter.setFlywheelSpeed(m_testingController.getShooterStick()*FlywheelConstants.kMaxSpeed);
      // })).whileFalse(Commands.runOnce(()->m_shooter.setFlywheelSpeed(0)));

      // m_testingController.maxSpeedShooter().onTrue(Commands.runOnce(()->{
      //   m_shooter.setFlywheelSpeed(FlywheelConstants.kMaxSpeed);
      // })).onFalse(Commands.runOnce(()->{
      //   m_shooter.setFlywheelSpeed(0);
      // }));
      // Commands.run(()->{
      //   m_shooter.setFlywheelSpeed(m_testingController.getShooterStick()*FlywheelConstants.kMaxSpeed);
      // }).schedule();

      m_testingController.intakeUpDegrees().onTrue(Commands.runOnce(()->m_intake.addDegreesToPivotAngle(2)));
      m_testingController.intakeDownDegrees().onTrue(Commands.runOnce(()->m_intake.addDegreesToPivotAngle(-2)));

      m_driverControls.goToIntakePosition().whileTrue(Commands.runOnce(()->{
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kIntake);
        m_shooter.isIntaking = ShooterIsIntaking.intaking;
      })).onFalse(Commands.runOnce(()->{
          if(m_robotState.curAction != RobotCurrentAction.kRevAndAlign){
            m_robotState.setRobotCurrentAction(RobotCurrentAction.kStow);
          }else{
            m_robotState.stowAndStopIntake();
          }
          Logger.recordOutput("stow Trigger", Timer.getFPGATimestamp());
          m_shooter.isIntaking = ShooterIsIntaking.notIntaking;
      }));

      m_driverControls.goToShootPositionAndRev().whileTrue(Commands.runOnce(()->{
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kRevAndAlign);
      })).onFalse(Commands.runOnce(()->{
        Logger.recordOutput("stow Trigger 2", Timer.getFPGATimestamp());
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kStow);
        m_robotState.setDriveType(DriveProfiles.kDefault);
      }));

      m_driverControls.fenderShot().onTrue(Commands.runOnce(()->{
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kShootFender);
      })).onFalse(Commands.runOnce(()->{
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kStow);
      }));

      // m_driverControls.ampAutoLineup().whileTrue(Commands.runOnce(()->{
      //   m_robotState.setRobotCurrentAction(RobotCurrentAction.kAmpLineup);
        
      //   // autoDriveCommand = m_autoFactory.trajectoryGenerateToPosition(FieldConstants.kAmpBlue,DriveConstants.kAutoAlignToAmpSpeed ,DriverStation.getAlliance().equals(Alliance.Red));
      //   // m_drive.setProfile(DriveProfiles.kTrajectoryFollowing);
        
      //   // autoDriveCommand.andThen(Commands.runOnce(()->{
      //   //   m_drive.setProfile(DriveProfiles.kDefault);
      //   // })).schedule();


      // }));

      m_driverControls.autoAlignToGamePiece().whileTrue(Commands.runOnce(()->{
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kGamePieceLock);
      })).onFalse(Commands.runOnce(()->{
        Logger.recordOutput("stow Trigger 3", Timer.getFPGATimestamp());
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kStow);
      }));
      m_driverControls.intakeVomit().whileTrue(Commands.runOnce(()->{
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kVomit);
      })).onFalse(Commands.runOnce(()->{
        Logger.recordOutput("stow Trigger 4", Timer.getFPGATimestamp());
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kStow);
      }));

      m_driverControls.autoIntake().whileTrue(Commands.runOnce(()->{
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kAutoIntake);
      }))
      .onFalse(Commands.runOnce(()->{
        m_drive.setProfile(DriveProfiles.kDefault);
        Logger.recordOutput("stow Trigger 5", Timer.getFPGATimestamp());
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kStow);
      }));
      // .onFalse(Commands.runOnce(()->{
      //   autoDriveCommand.cancel();
      //   m_autoFactory.cancel();
      //   // m_drive.drive(new ChassisSpeeds(0.0, 0.0,Rotation2d.fromDegrees(0).getRadians()));
      //   System.out.println("CANCELLING AUTO DRIVE");
      //   m_drive.setProfile(DriveProfiles.kDefault);
      // }));





  }

  public void planPathPlanner(){
    m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
    

    // Add basic autonomous commands
    m_autoChooser.addDefaultOption("Do Nothing", Commands.none());
    List<String> paths = PathPlannerUtil.getExistingPaths();
    for (String path : paths) {
      m_autoChooser.addOption(path, m_autoFactory.getAutoCommand(path));
    }
  }

  public void configureAutonomous(){
    m_autoFactory = new AutoFactory(m_drive, m_intake);
    planPathPlanner();
    
  }

  
  public void onDSConnected(){
    configureAutonomous();
  }

  public void configureSubsystems(){
    if (Robot.isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    m_climb = new Climb(new ClimbIOTalon(Ports.climbLeader, Ports.climbFollower, Ports.servoPort));
    m_intake = new Intake(new frc.robot.subsystems.intake.pivot.PivotIOSparkMax(Ports.wristMotorPort) , new frc.robot.subsystems.intake.rollers.RollerIOKraken(Ports.intakeMotorPort));
    // m_intake = new Intake(new frc.robot.subsystems.intake.pivot.PivotIOSim(), new frc.robot.subsystems.intake.rollers.RollerIOSim() );
    m_indexer = new Indexer(new frc.robot.subsystems.indexer.IndexerIOFalcon(Ports.indexerFirst,Ports.indexerSecond,Ports.beamBreakPort,Ports.beamBreakPort2));
    m_shooter = new Shooter(new PivotIOFalcon(Ports.shooterPivot, Ports.shooterPivotFollower,9 ), new FlywheelIOKraken(Ports.shooterLeft, Ports.shooterRight));
    // m_shooter = new Shooter(new frc.robot.subsystems.shooter.pivot.PivotIOSim(), new FlywheelIOKraken(Ports.shooterLeft, Ports.shooterRight));
    // Logger.recordOutput("kShooterBackLeft", new Pose3d(FieldConstants.kShooterBackLeft, new Rotation3d(0, 0, 0)));
    // Logger.recordOutput("kShooterBackRight", new Pose3d(FieldConstants.kShooterBackRight, new Rotation3d(0, 0, 0)));
    // Logger.recordOutput("kShooterFrontLeft", new Pose3d(FieldConstants.kShooterFrontLeft, new Rotation3d(0, 0, 0)));
    // Logger.recordOutput("kShooterFrontRight", new Pose3d(FieldConstants.kShooterFrontRight, new Rotation3d(0, 0, 0)));


    // Instantiate our RobotContainer. This will perform all our button bindings,

    

    
    
    // m_shooter = new Shooter(new PivotIOSim(), new FlywheelIOSim());
    // Unmanaged.setPhoenixDiagnosticsStartTime(-1);
    m_aprilTagVision = new frc.robot.subsystems.northstarAprilTagVision.AprilTagVision(new AprilTagVisionIONorthstar("northstar_0",""),new AprilTagVisionIONorthstar("northstar_1",""),new AprilTagVisionIONorthstar("northstar_2",""),new AprilTagVisionIONorthstar("northstar_3",""));
    if (Robot.isSimulation()) {
      SwerveModuleIO[] m_SwerveModuleIOs = {
        // new SwerveModuleIOMK4Talon(1,2,3),
        new SwerveModuleIOKraken(1,2,3,false),
        new SwerveModuleIOKraken(4,5, 6, false),
        new SwerveModuleIOKraken(7, 8, 9, false),
        new SwerveModuleIOKraken(10, 11, 12, false),
        // new SwerveModuleIOSim(),
        // new SwerveModuleIOMK4Talon(4,5,6),
        // new SwerveModuleIOSim(),
        // new SwerveModuleIOMK4Talon(7,8,9),
        // new SwerveModuleIOSim(),
        // new SwerveModuleIOMK4Talon(10,11,12),
      };
      m_drive = new Drive(new GyroIOPigeon(22, new Rotation2d(),true), new Pose2d(),m_SwerveModuleIOs );
          // new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim());
      // new SwerveModuleIOMK4Talon(1,2,3),
      //     new SwerveModuleIOMK4Talon(4,5,6),
      //     new SwerveModuleIOMK4Talon(7,8,9),
      //     new SwerveModuleIOMK4Talon(10,11,12));
      
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

        // SwerveModuleIO[] m_SwerveModuleIOs = {
        //   new SwerveModuleIOKraken(1,2,3,false),
        //   // new SwerveModuleIOMK4Talon(1,2,3),
        //   new SwerveModuleIOKraken(4,5, 6, false),
        //   // new SwerveModuleIOSim(),
        //   new SwerveModuleIOKraken(7, 8, 9, false),
        //   // new SwerveModuleIOMK4Talon(4,5,6),
        //   new SwerveModuleIOKraken(10, 11, 12, false),
        //   // new SwerveModuleIOSim(),
        //   // new SwerveModuleIOMK4Talon(7,8,9),
        //   // new SwerveModuleIOSim(),
        //   // new SwerveModuleIOMK4Talon(10,11,12),
        // };
        SwerveModuleIO[] m_SwerveModuleIOs = {
          // new SwerveModuleIOMK4Talon(1,2,3),
          new SwerveModuleIOKraken(10, 11, 12, false),
          // new SwerveModuleIOSim(),
          new SwerveModuleIOKraken(7, 8, 9, false),
          // new SwerveModuleIOSim(),
          new SwerveModuleIOKraken(4,5, 6, false),
          // new SwerveModuleIOSim(),
          new SwerveModuleIOKraken(1,2,3,false),
          // new SwerveModuleIOMK4Talon(4,5,6),
          // new SwerveModuleIOMK4Talon(7,8,9),
          // new SwerveModuleIOMK4Talon(10,11,12),
        };
        m_drive = new Drive(new GyroIOPigeon(22,new Rotation2d(),true),new Pose2d(),m_SwerveModuleIOs);
      // m_aprilTagVision = new frc.robot.subsystems.northstarAprilTagVision.AprilTagVision(new AprilTagVisionIONorthstar("northstar_0",frc.robot.subsystems.northstarAprilTagVision.AprilTagVisionConstants.cameraIds[0]));
    }

    m_objectDetectionCams = new ObjectDetectionCam(new ObjectDetectionIODepth(""));
    
    m_robotState = RobotState.startInstance(m_drive, m_climb, m_indexer,m_shooter, m_objectDetectionCams, m_aprilTagVision, m_intake,this::getAutoFactory,m_driverControls);
    // m_TebInterfacer = new NetworkTablesTEBInterfacer("teb");
  }

  public void updateRobotState(){
    VirtualSubsystem.periodicAll(); 
    m_robotState.updateRobotState();
    // m_robotState.calculateShooterAngle();
    
    // m_TebInterfacer.update();
    m_robotState.getEstimatedPose();
    // System.out.println("Drive chassis speeds: "+m_drive.getChassisSpeeds());
  }

  public AutoFactory getAutoFactory(){
    return m_autoFactory;
  }

  public void onDisabled(){
    m_drive.setWheelIdleBrake(true);
  }

  public void disabledPeriodic(){
    
  }

  public void onEnabled(){
    m_robotState.updateTestScheduler();
    if(edu.wpi.first.wpilibj.RobotState.isTeleop()){
      m_drive.setProfile(DriveProfiles.kDefault);
      
      m_drive.drive(new ChassisSpeeds(0,0,0));
      new TeleopControllerNoAugmentation(m_drive,()->m_driverControls.getDriveForward(),()->m_driverControls.getDriveLeft() , ()-> m_driverControls.getDriveRotation(), DriveConstants.controllerDeadzone).schedule();

    }
    
  }
  private void configureBindings() {
    // Transform3d cameraToObject =  new Transform3d(new Translation3d(2,3,0), new Rotation3d());
    // Transform3d robotToCamera = new Transform3d(new Translation3d(0,0,1), new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(30),Units.degreesToRadians(0)));
    // // advantagekit log
    // Pose3d objectPose = new Pose3d().plus(robotToCamera).plus(cameraToObject);
    // Logger.recordOutput("objectPose", objectPose); 
    // Logger.recordOutput("Camera Pose", new Pose3d().plus(robotToCamera));




  }

  public Command getAutonomousCommand() {
    return m_autoChooser.get();
    // return Commands.print("No autonomous command configured");
  }
}
