package frc.robot.subsystems.shooter;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.lib.hardwareprofiler.ProfiledSubsystem;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants.PivotConstants;
import frc.robot.subsystems.shooter.pivot.PivotIOInputsAutoLogged;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelInputsAutoLogged;
import frc.robot.subsystems.shooter.pivot.PivotIO;
import frc.robot.subsystems.shooter.pivot.PivotIO.PivotIOInputs;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends ProfiledSubsystem {
    PivotIO m_pivotIO;
    FlywheelIO m_flywheelIO;
    public PivotIOInputsAutoLogged m_pivotInputs;
    public FlywheelInputsAutoLogged m_flywheelInputs;
    ProfiledPIDController m_controller;


    public Shooter(PivotIO pivotIO, ProfiledPIDController controller, FlywheelIO flywheelIO, double tolerance) {
        super();
        m_pivotIO = pivotIO;
        m_pivotInputs = new PivotIOInputsAutoLogged();
        m_flywheelIO = flywheelIO;
        m_controller = controller;
        m_controller.setTolerance(tolerance);
        m_flywheelInputs = new FlywheelInputsAutoLogged();

    }
        
    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void periodic() {
        m_pivotIO.updateInputs(m_pivotInputs);
        m_flywheelIO.updateInputs(m_flywheelInputs);
        double pidVoltage = m_controller.calculate(m_flywheelInputs.velocityMetersPerSec);
        m_flywheelIO.setVoltage(pidVoltage);
        Logger.processInputs("Shooter/Flywheel", m_flywheelInputs); 
        Logger.processInputs("Shooter/Pivot", m_pivotInputs);
        Logger.recordOutput("Shooter/Flywheel/voltage", pidVoltage);
        Logger.recordOutput("Shooter/Flywheel/currentSetpoint", m_controller.getGoal().position);


    }

    public void setPivotAngle(Rotation2d angle) {
        if (Robot.isSimulation()) {
            angle = angle.minus(PivotConstants.simOffset);
        }
        // System.out.println("Setting angle to: "+angle.getDegrees());
        m_pivotIO.setDesiredAngle(angle);
    }

    public Transform3d getTransform() {
        // Rotation2d m_rotation = Rotation2d.fromDegrees(90).minus(Rotation2d.fromDegrees(m_inputs.curAngle));
        if (Robot.isSimulation()) {
            return new Transform3d(-0.23+.287, 0, 0.4, new Rotation3d(0, -m_pivotInputs.curAngle, 0));
        }
        return new Transform3d(-0.23+.287, 0, 0.4, new Rotation3d(0, m_pivotInputs.curAngle, 0));
      }
        // return new Transform3d(-0, 0, 0, new Rotation3d(0, m_rotation.getRadians(), 0));
    
  public void setVelocityMetersPerSecond(double velocity) {
    m_controller.setGoal(velocity);
  }

  public void setVelocityRadiansPerSecond(double velocity) {
    setVelocityMetersPerSecond(velocity * m_flywheelIO.getWheelLength());
  }

  public void setRevolutionsPerMinute(double rev) {
    setVelocityRadiansPerSecond(rev / (30 / Math.PI));
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public Command setVelocityCommand(double velocityMetersPerSec) {
    return runOnce(() -> setVelocityMetersPerSecond(velocityMetersPerSec));
  }

    
}
