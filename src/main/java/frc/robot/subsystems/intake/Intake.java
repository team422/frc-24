package frc.robot.subsystems.intake;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.hardwareprofiler.ProfiledSubsystem;
import frc.robot.subsystems.intake.pivot.PivotIO;
import frc.robot.subsystems.intake.pivot.PivotIOInputsAutoLogged;

public class Intake extends ProfiledSubsystem {
    
    PivotIO m_PivotIO;

    PivotIOInputsAutoLogged m_inputs;

    public Intake(PivotIO io) {
        super();
        m_PivotIO = io;
        m_inputs = new PivotIOInputsAutoLogged();
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void periodic() {
        m_PivotIO.updateInputs(m_inputs);

    }


    public void setPivotAngle(Rotation2d angle) {
        m_PivotIO.setDesiredAngle(angle);
    }
 
    public Transform3d getTransform() {
        return new Transform3d(-0.515+.287, 0, 0.233, new Rotation3d(0, m_inputs.curAngle, 0));
        // return new Transform3d(-0, 0, 0, new Rotation3d(0, m_rotation.getRadians(), 0));
    }
}
