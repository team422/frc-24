package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utils.EricNubControls;
import frc.robot.RobotState;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class TeleopControllerNoAugmentation extends Command {
    Drive m_drive;
  Supplier<Double> xSpeed;
  Supplier<Double> ySpeed;
  Supplier<Double> zRotation;
  double curXSpeed;
  double curYSpeed;
  double curZRotation;
  double deadzone;
  ChassisSpeeds speeds;
  EricNubControls m_controlsHandler;


  public TeleopControllerNoAugmentation(Drive drive, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
      Supplier<Double> zRotation, double deadzone) {
    m_drive = drive;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zRotation = zRotation;
    this.deadzone = deadzone;
    m_controlsHandler = new EricNubControls();
    addRequirements(drive);
  }

  public void execute() {
    curXSpeed = xSpeed.get();
    curYSpeed = ySpeed.get();
    curZRotation = zRotation.get();
    curXSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    curYSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    curZRotation *= DriveConstants.kMaxAngularSpeedRadiansPerSecond;

    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(curXSpeed, curYSpeed, curZRotation,
        m_drive.getPose().getRotation());
    m_drive.drive(speeds);
  }
}
