package frc.robot.commands.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utils.EricNubControls;
import frc.lib.utils.LoggedTunableNumber;
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
  
  PIDController mTurnController = new PIDController(1,0,0);
  double headingPIDActiveTime = -1;
  double headingTarget;
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
    mTurnController.enableContinuousInput(-Math.PI, Math.PI);
    headingTarget = RobotState.getInstance().getEstimatedPose().getRotation().getRadians();
    // addRequirements(drive);
  }
  @Override
  public void execute() {
    LoggedTunableNumber.ifChanged(hashCode(),()->{
      mTurnController.setPID(DriveConstants.kHeadingP.get(),DriveConstants.kHeadingI.get(),DriveConstants.kHeadingD.get());
    },DriveConstants.kHeadingP,DriveConstants.kHeadingI,DriveConstants.kHeadingD  );
    curXSpeed = xSpeed.get();
    curYSpeed = ySpeed.get();
    curZRotation = zRotation.get();
    curXSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    curYSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    curZRotation *= DriveConstants.kMaxAngularSpeedRadiansPerSecond;
    // System.out.println("Running");
    if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            curXSpeed *= -1;
            curYSpeed *= -1;
    }
    
    // System.out.println(speeds);
    Logger.recordOutput("Heading pid active",false);
    Logger.recordOutput("current controller z rotation",curZRotation);
    // if(headingPIDActiveTime < Timer.getFPGATimestamp()){

    //   if(curZRotation != 0){
    //     headingPIDActiveTime = Timer.getFPGATimestamp() + 1;
    //     headingTarget = m_drive.getPose().getRotation().getRadians();
    //   }
    //   if(Units.radiansToDegrees(headingTarget - m_drive.getPose().getRotation().getRadians()) > 25){
    //     headingTarget = m_drive.getPose().getRotation().getRadians();
        
    //   }
    //   else if(curZRotation == 0){
    //     curZRotation = mTurnController.calculate(m_drive.getPose().getRotation().getRadians(), headingTarget);
    //     Logger.recordOutput("Heading pid active",true);
    //   } 
    // }else {
    //   headingTarget = m_drive.getPose().getRotation().getRadians();
    // }
    Logger.recordOutput("current heading target",headingTarget);
    Logger.recordOutput("current curZRotation",curZRotation);

    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(curXSpeed, curYSpeed, curZRotation,
        m_drive.getPose().getRotation());
    m_drive.drive(speeds);
  }
}
