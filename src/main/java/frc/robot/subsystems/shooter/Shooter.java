package frc.robot.subsystems.shooter;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import frc.lib.hardwareprofiler.ProfiledSubsystem;
import frc.lib.utils.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.FlywheelConstants;
import frc.robot.Constants.ShooterConstants.ShooterPivotConstants;
import frc.robot.subsystems.shooter.pivot.PivotIOInputsAutoLogged;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.shooter.pivot.PivotIO;
import frc.robot.subsystems.shooter.pivot.PivotIO.PivotIOInputs;

public class Shooter extends ProfiledSubsystem {
    PivotIO m_pivotIO;
    FlywheelIO m_flywheelIO;
    PivotIOInputsAutoLogged m_inputsPivot;
    FlywheelIOInputsAutoLogged m_inputsFlywheel;

    TrapezoidProfile motionProfile;
    TrapezoidProfile.State setpointState;
    ArmFeedforward ff;

    


    double m_desiredSpeed; // m/s
    Rotation2d m_desiredAngle;

    public enum ShooterIsIntaking {
        intaking,notIntaking

    }
    public ShooterIsIntaking isIntaking;



    public Shooter(PivotIO pivotIO, FlywheelIO flywheelIO) {
        super();
        m_pivotIO = pivotIO;
        m_flywheelIO = flywheelIO;
        m_desiredAngle = ShooterPivotConstants.homeAngle;
        setpointState = new TrapezoidProfile.State(m_desiredAngle.getRadians(), 0);
          motionProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(Constants.ShooterConstants.ShooterPivotConstants.maxSpeed, Constants.ShooterConstants.ShooterPivotConstants.maxAcceleration));
        ff = new ArmFeedforward(ShooterPivotConstants.kPivotkS.get(),ShooterPivotConstants.kPivotkG.get(), ShooterPivotConstants.kPivotkV.get(), ShooterPivotConstants.kPivotkA.get());
        
        m_inputsPivot = new PivotIOInputsAutoLogged();
        m_inputsFlywheel = new FlywheelIOInputsAutoLogged();
    }

    



    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void periodic() {
        m_pivotIO.updateInputs(m_inputsPivot);
        m_flywheelIO.updateInputs(m_inputsFlywheel);
        
        
        Logger.processInputs("Shooter Pivot", m_inputsPivot);
        Logger.processInputs("Shooter Flywheel", m_inputsFlywheel); 
        State curState = new TrapezoidProfile.State(m_pivotIO.getCurrentAngle().getRadians(), m_pivotIO.getCurrentVelocity());
        setpointState =
          motionProfile.calculate(
              Constants.loopPeriodSecs,
              curState,
              new TrapezoidProfile.State(
                  MathUtil.clamp(
                      m_desiredAngle.getRadians(),
                      ShooterPivotConstants.minAngle.getRadians(),
                      ShooterPivotConstants.maxAngle.getRadians()),
                  0.0));

        // m_pivotIO.setDesiredAngle(angle);
        Logger.recordOutput("ShooterFF",ff.calculate(setpointState.position, setpointState.velocity));
        Logger.recordOutput("ShooterPosition",setpointState.position);
            if(isIntaking == ShooterIsIntaking.intaking) {
                m_pivotIO.runSetpoint(ShooterPivotConstants.homeAngle, 0);

            }
            else if (Constants.fullManualShooterAndPivotSpeedControls){
                Rotation2d m_ang = Rotation2d.fromRadians(MathUtil.clamp(
                Rotation2d.fromDegrees(ShooterPivotConstants.kShooterAngle.get()).getRadians(),
                      ShooterPivotConstants.minAngle.getRadians(),
                      ShooterPivotConstants.maxAngle.getRadians()));
            m_pivotIO.runSetpoint(m_ang, 0);
            // m_flywheelIO.setDesiredSpeed(FlywheelConstants.kFlywheelSpeed.get());
            m_flywheelIO.setDesiredSpeedWithSpin(FlywheelConstants.kFlywheelSpeedLeft.get(),FlywheelConstants.kFlywheelSpeedRight.get());
            }
            else{
                m_pivotIO.runSetpoint(m_desiredAngle, ff.calculate(setpointState.position, setpointState.velocity));
            }
    }

    public void setPivotAngle(Rotation2d angle) {
        if (Robot.isSimulation()) {
            angle = angle.minus(ShooterPivotConstants.simOffset);
        }
        // System.out.println("Setting angle to: "+angle.getDegrees());
        m_desiredAngle = angle;
        
    }

    public boolean isPivotWithinTolerance(Rotation2d angle,Rotation2d tolerance){
        return Math.abs(m_desiredAngle.minus(angle).getDegrees()) < tolerance.getDegrees();
    }

    public void setFlywheelSpeed(double speed) {
        m_flywheelIO.setDesiredSpeed(speed);
    }

    public void setFlywheelSpeedWithSpin(double speedLeft,double speedRight){
        m_flywheelIO.setDesiredSpeedWithSpin(speedLeft, speedRight);
    }

    

    public Rotation2d getPivotAngle() {
        return m_pivotIO.getDesiredAngle();
    }

    public Transform3d getTransform() {
        // Rotation2d m_rotation = Rotation2d.fromDegrees(90).minus(Rotation2d.fromDegrees(m_inputs.curAngle));
        if (Robot.isSimulation()) {
            return new Transform3d(-0.23+.287, 0, 0.4, new Rotation3d(0, -m_inputsPivot.curAngle, 0));
        }
        return new Transform3d(-0.23+.287, 0, 0.4, new Rotation3d(0, m_inputsPivot.curAngle, 0));
        // return new Transform3d(-0, 0, 0, new Rotation3d(0, m_rotation.getRadians(), 0));
    }

    public boolean isWithinTolerance(double speed) {
        if(m_inputsFlywheel.curSpeed - speed > 0){
            return true;
        }
        return Math.abs(m_inputsFlywheel.curSpeed - speed) < IntakeConstants.kFlywheelTolerance;

    }

    // public boolean isWithinToleranceWithSpin(double speedLeft,double speedRight){

    //     if(m_inputsFlywheel. - )

    // }
}
