package frc.robot.commands.autonomous;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotCurrentAction;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveProfiles;
import frc.robot.subsystems.indexer.Indexer.IndexerState;
import frc.robot.subsystems.intake.Intake;

public class AutoFactory extends Command {
  public static final PIDConstants linearPIDConstants = new PIDConstants(10, 0, 0);
  public static final PIDConstants angularPIDConstants = new PIDConstants(4, .3, 0);

  private final Drive m_drive;
  private final Intake m_intake;
  PathPlannerPath auto;

  public AutoFactory(Drive drive, Intake intake) {
    m_drive = drive;
    m_intake = intake;
    NamedCommands.registerCommand("AutoShoot", Commands.runOnce(()->{
      RobotState.getInstance().setIndexer(IndexerState.INDEXING);
    }).andThen(new AutoShoot()).andThen(Commands.runOnce(()->{
      // RobotState.getInstance().goToIntakePosition();
      // m_intake.setIntakeSpeed(IntakeConstants.intakeSpeed);
    })));
    NamedCommands.registerCommand("AutoSOTM", Commands.runOnce(()->{
      RobotState.getInstance().setIndexer(IndexerState.INDEXING);
    }).andThen(
      // Commands.none())
      new AutoSOTM()) 
      );
    NamedCommands.registerCommand("AutoFender", Commands.runOnce(()->{
      RobotState.getInstance().setIndexer(IndexerState.INDEXING);
    }).andThen(
      // Commands.none())
      new AutoFender()) 
      );
    NamedCommands.registerCommand("AutoIntake",
    // Commands.none()
     new AutoIntake() 
     );
    NamedCommands.registerCommand("stow", Commands.run(()->{
      RobotState.getInstance().stowAndStopIntake();
    }));

    NamedCommands.registerCommand("Align",Commands.runOnce(()->{
      RobotState.getInstance().setRobotCurrentAction(RobotCurrentAction.kAutoRevAndAutoAlign);

    }));
    NamedCommands.registerCommand("StayStill",Commands.runOnce(()->{
      RobotState.getInstance().getDrive().setProfile(DriveProfiles.kDefault);
      RobotState.getInstance().setRobotCurrentAction(RobotCurrentAction.kStow);
      RobotState.getInstance().getDrive().drive(new ChassisSpeeds(0,0,0));
    }));

    NamedCommands.registerCommand("Shoot", Commands.runOnce(()->{
      RobotState.getInstance().setIndexer(IndexerState.SHOOTING);
    }));

    NamedCommands.registerCommand("deployIntake",Commands.runOnce(()->{
      RobotState.getInstance().goToIntakePosition();
      m_intake.setIntakeSpeed(IntakeConstants.intakeSpeed);
      RobotState.getInstance().setRobotCurrentAction(RobotCurrentAction.kPathPlanner);
      RobotState.getInstance().setIndexer(IndexerState.INTAKING);
      RobotState.getInstance().setShooterSpeed(0);
    }));
    NamedCommands.registerCommand("2SecondLimit",Commands.waitSeconds(1).andThen(Commands.runOnce(()->{
      RobotState.getInstance().setRobotCurrentAction(RobotCurrentAction.kPathPlanner);
      RobotState.getInstance().getDrive().setProfile(DriveProfiles.kTrajectoryFollowing);
    })));

    NamedCommands.registerCommand("revCommand",Commands.none());
    NamedCommands.registerCommand("ShootEnd", Commands.run(()->{
      if(DriverStation.getMatchTime() < 0.5){
        RobotState.getInstance().setIndexer(IndexerState.SHOOTING);
      }
    }));
    NamedCommands.registerCommand("ShootFinal", Commands.runOnce(()->{
        RobotState.getInstance().setRobotCurrentAction(RobotCurrentAction.kNothing);
        RobotState.getInstance().m_shooter.setPivotAngle(Rotation2d.fromDegrees(28.5));
        RobotState.getInstance().m_shooter.setFlywheelSpeedWithSpin(11.0, 14.0);
        
        // RobotState.getInstance().setIndexer(IndexerState.SHOOTING);
    }));

    NamedCommands.registerCommand("fullSOTMClose",Commands.runOnce(()->{
        RobotState.getInstance().setRobotCurrentAction(RobotCurrentAction.kAutoShootClose);
    }));
    NamedCommands.registerCommand("AutoIntakeScanLeft",Commands.runOnce(()->{
      if(DriverStation.getAlliance().get().equals(Alliance.Red)){
        RobotState.getInstance().autoIntakeLeftOrRight = -1;
      }else{
        RobotState.getInstance().autoIntakeLeftOrRight = 1;
      }
      // RobotState.getInstance().setRobotCurrentAction(RobotCurrentAction.kAutoAutoIntake);
    }).andThen(new AutoIntake()));
    NamedCommands.registerCommand("AutoIntakeScanLeftDeadline",new AutoIntakeDeadline(true).withTimeout(1.0));

    NamedCommands.registerCommand("AutoIntakeScanRight",Commands.runOnce(()->{
      if(DriverStation.getAlliance().get().equals(Alliance.Red)){
        RobotState.getInstance().autoIntakeLeftOrRight = 1;
      }else{
        RobotState.getInstance().autoIntakeLeftOrRight = -1;
      }
      // RobotState.getInstance().setRobotCurrentAction(RobotCurrentAction.kAutoAutoIntake);
    }).andThen(new AutoIntake()));
    


    AutoBuilder.configureHolonomic(
        m_drive::getPose, // Robot pose supplier
        (Pose2d pose)->{},
        m_drive::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        m_drive::driveAuto, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(2.5, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(4, 1.6, 0.0), // Rotation PID constants
                    5.6, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig(true,true,1.7,1 ) // default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            m_drive // The subsystem that will be used to follow the path
    );
            PPHolonomicDriveController.setRotationTargetOverride(() -> RobotState.getInstance().getOptionalTargetOverride());
            PathPlannerLogging.setLogActivePathCallback((path) -> RobotState.getInstance().saveActivePath(path));
            PathPlannerLogging.setLogCurrentPoseCallback((pose) -> RobotState.getInstance().saveCurrentPose(pose));
            PathPlannerLogging.setLogTargetPoseCallback((pose) -> RobotState.getInstance().saveCurrentTarget(pose));
  }

  public PathPlannerPath loadPathGroupByName(String name) {
    return PathPlannerPath.fromPathFile(name);
  }

  public List<PathPlannerPath> getAutoTrajectory(String name) {
    return PathPlannerAuto.getPathGroupFromAutoFile(name);
  }

  

  public Command getAutoCommand(String nameString) {
    Command autoCommand = AutoBuilder.buildAuto(nameString);
    
    return autoCommand.andThen(m_drive.brakeCommand());
    // return Commands.sequence(m_drive.brakeCommand(), autoCommand.andThen(m_drive.brakeCommand()));
  }


  public Command trajectoryGenerateToPosition(Pose2d finalPose, PathConstraints constraints, boolean flipped) {
    if (flipped) {
      return AutoBuilder.pathfindToPoseFlipped(finalPose, constraints,0.0);
    } else{
      
      return AutoBuilder.pathfindToPose(finalPose, constraints,0.0 );
    }
  }
  public PathfindHolonomic generateTrajectoryToPose(Pose2d finalPose, PathConstraints constraints, boolean flipped,RobotState robotState) {

    return new PathfindHolonomic(finalPose, constraints, ()->RobotState.getInstance().getEstimatedPose(), ()->RobotState.getInstance().getChassisSpeeds(), robotState::setDriveToPieceChassisSpeeds, new HolonomicPathFollowerConfig(5.2,Units.inchesToMeters(15.5) , new ReplanningConfig()));

  }
}
