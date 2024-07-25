// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.DriverPropertyInfo;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.advantagekit.LoggerUtil;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotState.RobotCurrentAction;
import frc.robot.subsystems.drive.Drive.DriveProfiles;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private Command testCommand;
  private RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Initialize the AdvantageKit Logger
    
    // System.out.println(new ObjectMapper().writeValueAsString(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()));
    // try {
    // //   // System.out.println(new ObjectMapper().writeValueAsString(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()));
    // } catch (JsonProcessingException e) {
    //   throw new RuntimeException("Failed to serialize AprilTag layout JSON for Northstar");
    // }
      

    if (Robot.isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    
    switch (Constants.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new RLOGServer());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new RLOGServer());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(true); // Run as fast as possible
        String logPath = "/Volumes/NO NAME/Log_24-04-20_08-50-32_e2.wpilog";
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"), 0.01));
        break;
    }
    LoggerUtil.initializeLogger();
    PathfindingCommand.warmupCommand();
    robotContainer = new RobotContainer();
    robotContainer.lowerCurrentLimits();
  }

  @Override
  public void driverStationConnected(){
      robotContainer.onDSConnected();
  }

  /** This function is called periodically during all modes. */

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    long start = HALUtil.getFPGATime();
    CommandScheduler.getInstance().run();
    // CommandScheduler.getInstance().getActiveButtonLoop().poll();
    
    Logger.recordOutput("LoggedRobot/CommandScheduler", (HALUtil.getFPGATime()-start)/1000);
    robotContainer.updateRobotState();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {

    robotContainer.onDisabled();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    robotContainer.disabledPeriodic();

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    robotContainer.onEnabled();
    RobotState.getInstance().setDriveType(DriveProfiles.kTrajectoryFollowing);
    autonomousCommand = robotContainer.getAutonomousCommand();
    // autonomousCommand = Commands.none();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousExit(){
    if(DriverStation.isFMSAttached()){
      
      
    }
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    robotContainer.onEnabled();
    RobotState.getInstance().setDriveType(DriveProfiles.kDefault);
    RobotState.getInstance().setRobotCurrentAction(RobotCurrentAction.kStow);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // robotContainer.checkLock();
    RobotState.getInstance().updateTestScheduler();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    LiveWindow.setEnabled(false);
    robotContainer.onEnabled();
    // robotContainer.runTestSuite();
    // if (testCommand != null) {
    //   testCommand.schedule();
    // } else {
    //   System.out.println("No test command");
    // }

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
