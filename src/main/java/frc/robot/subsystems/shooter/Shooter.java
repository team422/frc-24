package frc.robot.subsystems.shooter;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.hardwareprofiler.ProfiledSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;
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


    public Shooter(PivotIO pivotIO, FlywheelIO flywheelIO) {
        super();
        m_pivotIO = pivotIO;
        m_flywheelIO = flywheelIO;
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

    }

    public void setPivotAngle(Rotation2d angle) {
        if (Robot.isSimulation()) {
            angle = angle.minus(ShooterPivotConstants.simOffset);
        }
        // System.out.println("Setting angle to: "+angle.getDegrees());
        setpointState =
          motionProfile.calculate(
              Constants.loopPeriodSecs,
              setpointState,
              new TrapezoidProfile.State(
                  MathUtil.clamp(
                      m_desiredAngle.getRadians(),
                      ShooterPivotConstants.minAngle.getRadians(),
                      ShooterPivotConstants.maxAngle.getRadians()),
                  0.0));

        // m_pivotIO.setDesiredAngle(angle);
        m_pivotIO.runSetpoint(angle, ff.calculate(setpointState.position, setpointState.velocity));
    }

    public void setFlywheelSpeed(double speed) {
        m_flywheelIO.setDesiredSpeed(speed);
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
}
