// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Setpoints;
import frc.robot.Constants.Vision.ClimbConstants;
import frc.robot.commands.drive.TeleopControllerNoAugmentation;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsXboxController;
import frc.robot.oi.OperatorControls;
import frc.robot.oi.OperatorControlsXbox;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveModuleIOSim;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.rollers.RollerIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.pivot.PivotIOSim;
import frc.robot.utils.ShooterMath;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;

public class RobotContainer {

  RobotState m_robotState;
  Drive m_drive;
  Intake m_intake;
  Shooter m_shooter;
  Climb m_climber;
  Flywheel m_flywheel;

  DriverControls m_driverControls;
  OperatorControls m_operatorControls;

  public RobotContainer() {
    configureSubsystems();
    configureBindings();
    configureCommands();

  }

  public void configureCommands() {
    m_drive.setDefaultCommand(new TeleopControllerNoAugmentation(m_drive, () -> m_driverControls.getDriveForward(),
        () -> m_driverControls.getDriveLeft(), () -> m_driverControls.getDriveRotation(),
        DriveConstants.controllerDeadzone));
    m_driverControls.setShooter45().onTrue(Commands.runOnce(() -> {
      System.out.println("Setting shooter to 45");
      m_shooter.setPivotAngle(Rotation2d.fromDegrees(45));
    }));

    m_operatorControls.setClimbTop().onTrue(m_climber.setDesiredHeightCommand(Setpoints.kClimberTop));
    m_operatorControls.setClimbBottom().onTrue(m_climber.setDesiredHeightCommand(Setpoints.kClimberBottom));
    m_operatorControls.climbUp().whileTrue(m_climber.moveCommand(ClimbConstants.kClimbUpSpeed.get()));
    m_operatorControls.climbDown().whileTrue(m_climber.moveCommand(-ClimbConstants.kClimbDownSpeed.get()));

    m_driverControls.manualShoot().onTrue(
      m_shooter.setVelocityCommand(8)
    );
  }

  public void configureSubsystems() {
    if (Robot.isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
      m_intake = new Intake(new frc.robot.subsystems.intake.pivot.PivotIOSim(),
                            new RollerIOSim(), IntakeConstants.kIntakeVoltage);
    }
    // Logger.recordOutput("kShooterBackLeft", new Pose3d(FieldConstants.kShooterBackLeft, new Rotation3d(0, 0, 0)));
    // Logger.recordOutput("kShooterBackRight", new Pose3d(FieldConstants.kShooterBackRight, new Rotation3d(0, 0, 0)));
    // Logger.recordOutput("kShooterFrontLeft", new Pose3d(FieldConstants.kShooterFrontLeft, new Rotation3d(0, 0, 0)));
    // Logger.recordOutput("kShooterFrontRight", new Pose3d(FieldConstants.kShooterFrontRight, new Rotation3d(0, 0, 0)));


    // Instantiate our RobotContainer. This will perform all our button bindings,

    m_drive = new Drive(new GyroIOPigeon(22, new Rotation2d()), new Pose2d(),
          new SwerveModuleIOSim(),
          new SwerveModuleIOSim(),
          new SwerveModuleIOSim(),
          new SwerveModuleIOSim());
    
    
    if (Robot.isSimulation()) {
      m_shooter = new Shooter(new PivotIOSim(), FlywheelConstants.flywheelController, new FlywheelIOSim(), FlywheelConstants.tolerance);
    }

    if (Robot.isSimulation()) {
      m_climber = new Climb(new ClimbIOSim(),
          new ProfiledPIDController(ClimbConstants.kClimbP.get(), ClimbConstants.kClimbI.get(),
              ClimbConstants.kClimbD.get(),
              new Constraints(ClimbConstants.kMaxVelocity, ClimbConstants.kMaxAcceleration)),
          ClimbConstants.kMinHeight, ClimbConstants.kMaxHeight);
    }

    m_robotState = RobotState.startInstance(m_drive, null, null, m_shooter, null, null, m_intake);
  }

  public void updateRobotState() {
    m_robotState.updateRobotState();
    m_robotState.calculateShooterAngle();
  }

  public void onDisabled() {

  }

  public void disabledPeriodic() {

  }

  public void onEnabled() {

  }

  private void configureBindings() {
    m_driverControls = new DriverControlsXboxController(4);
    m_operatorControls = new OperatorControlsXbox(5);
  }
  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

