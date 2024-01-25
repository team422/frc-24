package frc.robot.subsystems.shooter;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.hardwareprofiler.ProfiledSubsystem;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants.PivotConstants;
import frc.robot.subsystems.shooter.pivot.PivotIOInputsAutoLogged;
import frc.robot.subsystems.shooter.pivot.PivotIO;
import frc.robot.subsystems.shooter.pivot.PivotIO.PivotIOInputs;

public class Shooter extends ProfiledSubsystem {
    PivotIO m_pivotIO;
    PivotIOInputsAutoLogged m_inputs;
    public Shooter(PivotIO pivotIO) {
        super();
        m_pivotIO = pivotIO;
        m_inputs = new PivotIOInputsAutoLogged();
    }



    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void periodic() {
        m_pivotIO.updateInputs(m_inputs);

    }

    public void setPivotAngle(Rotation2d angle) {
        if (Robot.isSimulation()) {
            angle = angle.minus(PivotConstants.simOffset);
        }
        System.out.println("Setting angle to: "+angle.getDegrees());
        m_pivotIO.setDesiredAngle(angle);
    }

    public Transform3d getTransform() {
        // Rotation2d m_rotation = Rotation2d.fromDegrees(90).minus(Rotation2d.fromDegrees(m_inputs.curAngle));
        if (Robot.isSimulation()) {
            return new Transform3d(-0.23+.287, 0, 0.4, new Rotation3d(0, -m_inputs.curAngle, 0));
        }
        return new Transform3d(-0.23+.287, 0, 0.4, new Rotation3d(0, m_inputs.curAngle, 0));
        // return new Transform3d(-0, 0, 0, new Rotation3d(0, m_rotation.getRadians(), 0));
    }
}
