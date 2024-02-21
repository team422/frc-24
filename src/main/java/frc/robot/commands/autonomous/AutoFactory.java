package frc.robot.commands.autonomous;

import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.utils.FieldGeomUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;

public class AutoFactory extends Command {
  public static final PIDConstants linearPIDConstants = new PIDConstants(10, 0, 0);
  public static final PIDConstants angularPIDConstants = new PIDConstants(4, 0, 0);

  private final Drive m_drive;
  private final Intake m_intake;

  public AutoFactory(Drive drive, Intake intake) {
    m_drive = drive;
    m_intake = intake;
  }

  public PathPlannerPath loadPathGroupByName(String name) {
    return PathPlannerPath.fromPathFile(name);
  }

  public List<PathPlannerPath> getAutoTrajectory(String name) {
    return PathPlannerAuto.getPathGroupFromAutoFile(name);

  }

  public Command getAutoCommand(String nameString) {
    // Create Auto builder
    // AutoBuilder.configureHolonomic(
    //     m_drive::getPose,
    //     m_drive::resetPose,
    //     m_drive::getChassisSpeeds,
    //     m_drive::drive,
    //     new HolonomicPathFollowerConfig(linearPIDConstants, angularPIDConstants,
    //         DriveConstants.kMaxModuleSpeedMetersPerSecond, DriveConstants.kTrackWidth, new ReplanningConfig()),
    //     m_drive);
    AutoBuilder.configureHolonomic(
        m_drive::getPose, // Robot pose supplier
        m_drive::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        m_drive::getChassisSpeedsRobotRelative, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        m_drive::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    5.4, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
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

    PathPlannerPath auto = PathPlannerPath.fromChoreoTrajectory(nameString);
    Command autoCommand = AutoBuilder.followPath(auto);

    // return autoCommand.andThen(m_drive.brakeCommand());
    return Commands.sequence(m_drive.resetCommand(auto.getPreviewStartingHolonomicPose()),m_drive.brakeCommand(), autoCommand.andThen(m_drive.brakeCommand()));
  }
}
