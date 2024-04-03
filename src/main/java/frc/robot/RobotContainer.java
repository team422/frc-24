// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.unmanaged.Unmanaged;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.autonPlanning.PathPlannerUtil;
import frc.lib.utils.NetworkTablesTEBInterfacer;
import frc.lib.utils.VirtualSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Ports;
import frc.robot.RobotState.RobotCurrentAction;
import frc.robot.commands.autonomous.AutoFactory;
import frc.robot.commands.drive.FeedForwardCharacterization;
import frc.robot.commands.drive.TeleopControllerNoAugmentation;
import frc.robot.commands.drive.WheelRadiusCharacterization;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsXboxController;
import frc.robot.oi.ManualController;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.amp.AmpIOFalcon;
import frc.robot.subsystems.amp.Amp.AmpState;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOTalon;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveProfiles;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIOKraken;
import frc.robot.subsystems.drive.generatedConstants.TunerConstants;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.northstarAprilTagVision.SimVisionSystem;
import frc.robot.subsystems.objectVision.ObjectDetectionCam;
import frc.robot.subsystems.objectVision.ObjectDetectionIODepth;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterIsIntaking;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOKraken;
import frc.robot.subsystems.shooter.pivot.PivotIOFalcon;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.AutoBuilderManager;
import frc.robot.utils.AutoScanLoop;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.Note;
import frc.robot.utils.NoteVisualizer;

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
  Led m_led;
  SimVisionSystem[] m_simVisionSystems;
  
  DriverControls m_driverControls;
  ManualController m_testingController;

  ObjectDetectionCam m_objectDetectionCams;

  AutoScanLoop m_autoScanLoop = new AutoScanLoop();
  AutoBuilderManager m_autoBuilderManager = new AutoBuilderManager();

  Command autoDriveCommand;

  Amp m_amp;


  private LoggedDashboardChooser<Command> m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
  private LoggedDashboardChooser<Command> m_scoutingChooser = new LoggedDashboardChooser<>("Scouting Chooser");
  private LoggedDashboardChooser<Command> m_scoutNumber = new LoggedDashboardChooser<>("Scouting Number");
  private LoggedDashboardChooser<Command> m_endChooser = new LoggedDashboardChooser<>("End Chooser");

  public RobotContainer() {
    configureControllers();
    configureSubsystems();
    configureBindings();
    configureCommands();
    configureBindings();
  }

  public void configureControllers(){
    m_driverControls = new DriverControlsXboxController(1);
    // m_driverControls = new DriverControlsXboxReal(3);
    m_testingController = new ManualController(5);
    
  }

  public void configureCommands(){
    // m_drive.setDefaultCommand();
    
    m_driverControls.resetFieldCentric().onTrue(Commands.runOnce(()->m_drive.resetPose(new Pose2d(m_robotState.getEstimatedPose().getTranslation(),AllianceFlipUtil.apply(Rotation2d.fromDegrees(180))))));
  
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
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kAmpLineup);
      })).onFalse(Commands.runOnce(()->{
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kStow);
      }));
      m_testingController.finalShoot().onTrue(Commands.runOnce(()->{
        System.out.println("NOW SHOOT");
        m_indexer.setState(IndexerState.SHOOTING);
      }));

      m_driverControls.hockeyPuck().whileTrue(Commands.runOnce(()->{
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kHockeyPuck);
      })).onFalse(Commands.runOnce(()->{
        Logger.recordOutput("stow Trigger 1", Timer.getFPGATimestamp());
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kStow);
        m_robotState.setDriveType(DriveProfiles.kDefault);
      }));
      
      m_testingController.toggleBeamBreakOne().onTrue(
        Commands.runOnce(() -> m_indexer.io.setInitalBeamBreak(true))
      );
      m_testingController.toggleBeamBreakOne().onFalse(
        Commands.runOnce(() -> m_indexer.io.setInitalBeamBreak(false))
      );
      m_testingController.toggleBeamBreakTwo().onTrue(
        Commands.runOnce(() -> m_indexer.io.setFinalBeamBreak(true))
      );
      m_testingController.toggleBeamBreakTwo().onFalse(
        Commands.runOnce(() -> m_indexer.io.setFinalBeamBreak(false))
      );

      m_testingController.beambreakGamePiece().onTrue(
        Commands.runOnce(() -> m_indexer.io.gamepiece())
      );




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

      // m_testingController.intakeUpDegrees().onTrue(Commands.runOnce(()->m_intake.addDegreesToPivotAngle(2)));
      // m_testingController.intakeDownDegrees().onTrue(Commands.runOnce(()->m_intake.addDegreesToPivotAngle(-2)));
      m_testingController.sourceIntake().onTrue(Commands.runOnce(()->{
        RobotState.getInstance().setRobotCurrentAction(RobotCurrentAction.kSourceIntake);
      })).onFalse(Commands.runOnce(()->{
        RobotState.getInstance().setIndexer(IndexerState.IDLE);
        RobotState.getInstance().setRobotCurrentAction(RobotCurrentAction.kStow);
      }));

      m_driverControls.goToIntakePosition().whileTrue(Commands.runOnce(()->{
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kIntake);
        m_shooter.isIntaking = ShooterIsIntaking.intaking;
      })).onFalse(Commands.runOnce(()->{
          if(m_robotState.curAction != RobotCurrentAction.kRevAndAlign){
            m_robotState.setRobotCurrentAction(RobotCurrentAction.kStow);
            m_indexer.setState(IndexerState.IDLE);
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

      m_driverControls.ampAutoLineup().whileTrue(Commands.runOnce(()->{
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kAmpLineup);
        
        // autoDriveCommand = m_autoFactory.trajectoryGenerateToPosition(FieldConstants.kAmpBlue,DriveConstants.kAutoAlignToAmpSpeed ,DriverStation.getAlliance().equals(Alliance.Red));
        // m_drive.setProfile(DriveProfiles.kTrajectoryFollowing);
        
        // autoDriveCommand.andThen(Commands.runOnce(()->{
        //   m_drive.setProfile(DriveProfiles.kDefault);
        // })).schedule();


      })).onFalse(Commands.runOnce(()->{
        Logger.recordOutput("stow Trigger 13", Timer.getFPGATimestamp());
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kStow);
        m_robotState.setDriveType(DriveProfiles.kDefault);
      
      }));

      m_driverControls.autoAlignToGamePiece().whileTrue(Commands.runOnce(()->{
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kGamePieceLock);
      })).onFalse(Commands.runOnce(()->{
        Logger.recordOutput("stow Trigger 3", Timer.getFPGATimestamp());
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kStow);
      }));
      m_testingController.preRev().whileTrue(Commands.runOnce(()->{
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kRevAndAlign);
      })).onFalse(Commands.runOnce(()->{
        Logger.recordOutput("stow Trigger 22", Timer.getFPGATimestamp());
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kStow);
        m_robotState.setDriveType(DriveProfiles.kDefault);
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
      m_testingController.zeroAmp().whileTrue(Commands.runOnce((()->{
        m_intake.isAmpZeroing = true;
        
        m_amp.m_state = AmpState.Zeroing;
      }))).onFalse(Commands.runOnce(()->{
        m_intake.isAmpZeroing =false;
        m_amp.m_state = AmpState.PositionFollowing;
      }));
      m_driverControls.ampBackTrigger().whileTrue(Commands.runOnce(()->{
        m_indexer.setState(IndexerState.BACKTOINTAKE);
        m_robotState.setRobotCurrentAction(RobotCurrentAction.kNoteBackToIntake);
      })).onFalse(Commands.runOnce(()->{
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


      m_driverControls.testRumble().whileTrue(Commands.runEnd(
        () -> m_driverControls.setDriverRumble(0.2, RumbleType.kLeftRumble),
        () -> m_driverControls.setDriverRumble(0, RumbleType.kLeftRumble)));


  }

  public void addBasePaths(){
    m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
    

    // Add basic autonomous commands
    m_autoChooser.addDefaultOption("Do Nothing", Commands.none());
    List<String> paths = PathPlannerUtil.getExistingPaths();
    for (String path : paths) {
      m_autoChooser.addOption(path, m_autoFactory.getAutoCommand(path).andThen(Commands.runOnce(()->{
        // RobotState.getInstance().mUpdatingAutoBuilder = true;
      })));
    }
    m_autoChooser.addOption(
      "Drive Wheel Radius Characterization",
      m_drive
          .orientModules(Drive.getCircleOrientations())
          .andThen(
              new WheelRadiusCharacterization(
                  m_drive, WheelRadiusCharacterization.Direction.COUNTER_CLOCKWISE))
          .withName("Drive Wheel Radius Characterization"));
    m_autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
                m_drive, m_drive::runCharacterization, m_drive::getCharacterizationVelocity)
            .finallyDo(m_drive::endCharacterization));

  }

  public void configureAutonomous(){
    m_autoFactory = new AutoFactory(m_drive, m_intake);
    addBasePaths();
    addScoutingPathing();

  }


  public void addScoutingPathing(){
    m_scoutingChooser = new LoggedDashboardChooser<>("Scouting Chooser");
    m_endChooser = new LoggedDashboardChooser<>("End Chooser");
    m_scoutingChooser.addDefaultOption("None", Commands.runOnce(()->{
      RobotState.getInstance().getAutoBuilderManager().setNotes(new Note[0]);
    }));
    m_autoScanLoop.getAllScoutPossibilties().forEach((key, value) -> {

      m_scoutingChooser.addOption(key, Commands.runOnce(()->{ 
        RobotState.getInstance().getAutoBuilderManager().setNotes(value);
      }));
    });

    m_endChooser.addDefaultOption("None", Commands.runOnce(()->{ 
      RobotState.getInstance().getAutoBuilderManager().setEndNotes(new Note[0]);
    }));
    m_autoScanLoop.getAllEndPossibilties().forEach((key, value) -> {
      m_endChooser.addOption(key, Commands.runOnce(()->{ 
        RobotState.getInstance().getAutoBuilderManager().setEndNotes(value);
      }));
    });

    m_scoutNumber = new LoggedDashboardChooser<>("Scouting Number");
    m_scoutNumber.addDefaultOption("0", Commands.runOnce(()->{
      RobotState.getInstance().getAutoBuilderManager().setNumScouts(0);
    }));
    m_scoutNumber.addOption("1", Commands.runOnce(()->{
      RobotState.getInstance().getAutoBuilderManager().setNumScouts(1);
    }));
    m_scoutNumber.addOption("2", Commands.runOnce(()->{
      RobotState.getInstance().getAutoBuilderManager().setNumScouts(2);
    }));
    m_scoutNumber.addOption("3", Commands.runOnce(()->{
      RobotState.getInstance().getAutoBuilderManager().setNumScouts(3);
    }));
    m_scoutNumber.addOption("4", Commands.runOnce(()->{
      RobotState.getInstance().getAutoBuilderManager().setNumScouts(4);
    }));



    

  }

  
  public void onDSConnected(){
    configureAutonomous();
    if (Robot.isSimulation()) {
      m_simVisionSystems = new SimVisionSystem[4];
      for (int i = 0; i < 4; i++) {
        m_simVisionSystems[i] = new SimVisionSystem(
          "northstar_" + i,
          100.0,
          GeomUtil.toTransform3d(frc.robot.subsystems.northstarAprilTagVision.AprilTagVisionConstants.cameraPoses[i]).inverse(),
          6,
          1600,
          1200,
          0.2
        );
      }
    }
    
  }

  public void configureSubsystems(){
    if (Robot.isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    m_climb = new Climb(new ClimbIOTalon(Ports.climbLeader, Ports.climbFollower, Ports.servoPort));
    // m_intake = new Intake(new frc.robot.subsystems.intake.pivot.PivotIOSim(), new frc.robot.subsystems.intake.rollers.RollerIOSim() );
    m_indexer = new Indexer(new frc.robot.subsystems.indexer.IndexerIOFalcon(Ports.indexerFirst,Ports.indexerSecond,Ports.beamBreakPort,Ports.beamBreakPort2));
    
    // m_shooter = new Shooter(new frc.robot.subsystems.shooter.pivot.PivotIOSim(), new FlywheelIOKraken(Ports.shooterLeft, Ports.shooterRight));
    // Logger.recordOutput("kShooterBackLeft", new Pose3d(FieldConstants.kShooterBackLeft, new Rotation3d(0, 0, 0)));
    // Logger.recordOutput("kShooterBackRight", new Pose3d(FieldConstants.kShooterBackRight, new Rotation3d(0, 0, 0)));
    // Logger.recordOutput("kShooterFrontLeft", new Pose3d(FieldConstants.kShooterFrontLeft, new Rotation3d(0, 0, 0)));
    // Logger.recordOutput("kShooterFrontRight", new Pose3d(FieldConstants.kShooterFrontRight, new Rotation3d(0, 0, 0)));
    // m_led = new Led(Ports.ledPort, 20);

    

    // Instantiate our RobotContainer. This will perform all our button bindings,

    

    
    
    // m_shooter = new Shooter(new PivotIOSim(), new FlywheelIOSim());
    Unmanaged.setPhoenixDiagnosticsStartTime(-1);
    m_aprilTagVision = new frc.robot.subsystems.northstarAprilTagVision.AprilTagVision(new frc.robot.subsystems.northstarAprilTagVision.AprilTagVisionIONorthstar("northstar_0",""),new frc.robot.subsystems.northstarAprilTagVision.AprilTagVisionIONorthstar("northstar_1",""),new frc.robot.subsystems.northstarAprilTagVision.AprilTagVisionIONorthstar("northstar_2",""),new frc.robot.subsystems.northstarAprilTagVision.AprilTagVisionIONorthstar("northstar_3",""));
    if (Robot.isSimulation()) {
      m_intake = new Intake(new frc.robot.subsystems.intake.pivot.PivotIOSim() ,new frc.robot.subsystems.intake.rollers.RollerIOSim());
      
      m_shooter = new Shooter(new PivotIOFalcon(Ports.shooterPivot, Ports.shooterPivotFollower,9 ), new FlywheelIOKraken(Ports.shooterLeft, Ports.shooterRight));
      CommandSwerveDrivetrain m_CommandSwerveDrivetrain = new CommandSwerveDrivetrain(TunerConstants.DrivetrainConstants,0.005, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
      SwerveModuleIO[] m_SwerveModuleIOs = {
        new SwerveModuleIOKraken(m_CommandSwerveDrivetrain.getModule(0).getDriveMotor(),m_CommandSwerveDrivetrain.getModule(0).getSteerMotor(),m_CommandSwerveDrivetrain.getModule(0).getCANcoder(), false),
          // new SwerveModuleIOSim(),
          new SwerveModuleIOKraken(m_CommandSwerveDrivetrain.getModule(1).getDriveMotor(),m_CommandSwerveDrivetrain.getModule(1).getSteerMotor(),m_CommandSwerveDrivetrain.getModule(1).getCANcoder(), false),
          // new SwerveModuleIOKraken(7, 8, 9, false),
          // new SwerveModuleIOSim(),
          new SwerveModuleIOKraken(m_CommandSwerveDrivetrain.getModule(2).getDriveMotor(),m_CommandSwerveDrivetrain.getModule(2).getSteerMotor(),m_CommandSwerveDrivetrain.getModule(2).getCANcoder(), false),
          // new SwerveModuleIOKraken(4,5, 6, false),
          // new SwerveModuleIOSim(),
          new SwerveModuleIOKraken(m_CommandSwerveDrivetrain.getModule(3).getDriveMotor(),m_CommandSwerveDrivetrain.getModule(3).getSteerMotor(),m_CommandSwerveDrivetrain.getModule(3).getCANcoder(), false),
        // new SwerveModuleIOMK4Talon(1,2,3),
        // new SwerveModuleIOKraken(1,2,3,false),
        // new SwerveModuleIOKraken(4,5, 6, false),
        // new SwerveModuleIOKraken(7, 8, 9, false),
        // new SwerveModuleIOKraken(10, 11, 12, false),
        // new SwerveModuleIOSim(),
        // new SwerveModuleIOMK4Talon(4,5,6),
        // new SwerveModuleIOSim(),
        // new SwerveModuleIOMK4Talon(7,8,9),
        // new SwerveModuleIOSim(),
        // new SwerveModuleIOMK4Talon(10,11,12),
      };
      m_drive = new Drive(new GyroIOPigeon(22, new Rotation2d(),true), new Pose2d(),m_CommandSwerveDrivetrain,m_SwerveModuleIOs );
          // new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim());
      // new SwerveModuleIOMK4Talon(1,2,3),
      //     new SwerveModuleIOMK4Talon(4,5,6),
      //     new SwerveModuleIOMK4Talon(7,8,9),
      //     new SwerveModuleIOMK4Talon(10,11,12));
      m_amp = new Amp(new AmpIOFalcon(Ports.ampMotor));
      // new frc.robot.subsystems.intake.rollers.RollerIOKraken(Ports.intakeMotorPort)
    }
    else{
      m_intake = new Intake(new frc.robot.subsystems.intake.pivot.PivotIOSparkMax(Ports.wristMotorPort) ,new frc.robot.subsystems.intake.rollers.RollerIOKraken(Ports.intakeMotorPort) );
    m_shooter = new Shooter(new PivotIOFalcon(Ports.shooterPivot, Ports.shooterPivotFollower,9 ), new FlywheelIOKraken(Ports.shooterLeft, Ports.shooterRight));
    m_amp = new Amp(new AmpIOFalcon(Ports.ampMotor));
      // m_shooter = new Shooter(new frc.robot.subsystems.shooter.pivot.PivotIOSim(), new FlywheelIOSim());
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
        CommandSwerveDrivetrain m_CommandSwerveDrivetrain = new CommandSwerveDrivetrain(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
        SwerveModuleIO[] m_SwerveModuleIOs = {
          // new SwerveModuleIOMK4Talon(1,2,3),
          new SwerveModuleIOKraken(m_CommandSwerveDrivetrain.getModule(0).getDriveMotor(),m_CommandSwerveDrivetrain.getModule(0).getSteerMotor(),m_CommandSwerveDrivetrain.getModule(0).getCANcoder(), false),
          // new SwerveModuleIOSim(),
          new SwerveModuleIOKraken(m_CommandSwerveDrivetrain.getModule(1).getDriveMotor(),m_CommandSwerveDrivetrain.getModule(1).getSteerMotor(),m_CommandSwerveDrivetrain.getModule(1).getCANcoder(), false),
          // new SwerveModuleIOKraken(7, 8, 9, false),
          // new SwerveModuleIOSim(),
          new SwerveModuleIOKraken(m_CommandSwerveDrivetrain.getModule(2).getDriveMotor(),m_CommandSwerveDrivetrain.getModule(2).getSteerMotor(),m_CommandSwerveDrivetrain.getModule(2).getCANcoder(), false),
          // new SwerveModuleIOKraken(4,5, 6, false),
          // new SwerveModuleIOSim(),
          new SwerveModuleIOKraken(m_CommandSwerveDrivetrain.getModule(3).getDriveMotor(),m_CommandSwerveDrivetrain.getModule(3).getSteerMotor(),m_CommandSwerveDrivetrain.getModule(3).getCANcoder(), false),
          // new SwerveModuleIOKraken(1,2,3,false),
          // new SwerveModuleIOMK4Talon(4,5,6),
          // new SwerveModuleIOMK4Talon(7,8,9),
          // new SwerveModuleIOMK4Talon(10,11,12),
        };
        // SwerveModuleIO[] m_SwerveModuleIOs = {
        //   new SwerveModuleIOKraken(new TalonFX(1),new TalonFX(2),new CANcoder(6),false),
        //   new SwerveModuleIOKraken(new TalonFX(3),new TalonFX(4),new CANcoder(5),false), 
        //   new SwerveModuleIOKraken(new TalonFX(7),new TalonFX(8),new CANcoder(9),false),
        //   new SwerveModuleIOKraken(new TalonFX(10),new TalonFX(11),new CANcoder(12),false)

        // };
        m_drive = new Drive(new GyroIOPigeon(22,new Rotation2d(),true),new Pose2d(),m_CommandSwerveDrivetrain,m_SwerveModuleIOs);
      // m_aprilTagVision = new frc.robot.subsystems.northstarAprilTagVision.AprilTagVision(new frc.robot.subsystems.northstarAprilTagVision.AprilTagVisionIONorthstar("northstar_0",frc.robot.subsystems.northstarAprilTagVision.AprilTagVisionConstants.cameraIds[0]));
    }

    m_objectDetectionCams = new ObjectDetectionCam(new ObjectDetectionIODepth(""));
    
    m_robotState = RobotState.startInstance(m_drive, m_climb, m_indexer,m_shooter, m_objectDetectionCams, m_aprilTagVision, m_intake,this::getAutoFactory,m_driverControls,m_amp,m_autoBuilderManager);
    // m_TebInterfacer = new NetworkTablesTEBInterfacer("teb");
  }

  public void updateRobotState(){
    double start = HALUtil.getFPGATime();
    VirtualSubsystem.periodicAll(); 
    Logger.recordOutput("LoggedRobot/VirtualSubsystems", (HALUtil.getFPGATime()-start)/1000);
    start = HALUtil.getFPGATime();
    m_robotState.updateRobotState();
    // m_robotState.calculateShooterAngle();
    
    // m_TebInterfacer.update();
    m_robotState.getEstimatedPose();

    if(Robot.isSimulation()){
      updateNoteVisualizer();
    }
    Logger.recordOutput("LoggedRobot/RobotState", (HALUtil.getFPGATime()-start)/1000);
    // System.out.println("Drive chassis speeds: "+m_drive.getChassisSpeeds());
  }

  public void updateNoteVisualizer(){
    if (edu.wpi.first.wpilibj.RobotState.isAutonomous()){
    List<Translation2d> presentNotes = NoteVisualizer.getAutoNotes().stream().filter(Objects::nonNull).toList();
    System.out.println(presentNotes);
    for (Translation2d translation : presentNotes){
      if (translation.getDistance(m_robotState.getEstimatedPose().getTranslation()) < 0.5){
        NoteVisualizer.takeAutoNote(NoteVisualizer.autoNotes.indexOf(translation));
        NoteVisualizer.showAllNotes();
      }
    };
    NoteVisualizer.showHeldNotes();
  }
  else {
    // check if the robot is near a note
    List<Translation2d> presentNotes = NoteVisualizer.getAllNotes();
    for (Translation2d translation : presentNotes){
      if (translation.getDistance(m_robotState.getEstimatedPose().getTranslation()) < 0.5){
        NoteVisualizer.takeNote(translation);
      }
    };
    NoteVisualizer.showAllNotes();
    // check if note needs to be added
    NoteVisualizer.checkWhetherToAddNote();

  }
  }

  public AutoFactory getAutoFactory(){
    return m_autoFactory;
  }

  public void onDisabled(){
    m_drive.setWheelIdleBrake(true);
    RobotState.getInstance().getAutoBuilderManager().reset();
  }

  public void disabledPeriodic(){
    
  }

  public void onEnabled(){
    // m_robotState.updateTestScheduler();
    m_shooter.clearI();
    if(edu.wpi.first.wpilibj.RobotState.isTeleop()){
      m_drive.setProfile(DriveProfiles.kDefault);
      
      // Commands.runOnce(()->{
      //   m_amp.m_state = AmpState.Zeroing;
      // }).andThen(Commands.waitSeconds(1)).andThen(()->{
      //   m_amp.m_state = AmpState.PositionFollowing;
      // }).schedule();
      m_drive.drive(new ChassisSpeeds(0,0,0));
      new TeleopControllerNoAugmentation(m_drive,()->m_driverControls.getDriveForward(),()->m_driverControls.getDriveLeft() , ()-> m_driverControls.getDriveRotation(), DriveConstants.controllerDeadzone).schedule();

    }
    m_robotState.mUpdatingAutoBuilder = false;
    
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
    if(Robot.isSimulation()){
    NoteVisualizer.resetAutoNotes();
    NoteVisualizer.showAllNotes();
    }
    return m_autoChooser.get();
    // return m_scoutNumber.get().andThen(m_scoutingChooser.get()).andThen(m_endChooser.get()).andThen(m_autoChooser.get());
    // return Commands.print("No autonomous command configured");
  }
}
