package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.hardwareprofiler.ProfiledSubsystem;
import frc.robot.subsystems.intake.pivot.IntakePivotIO;
import frc.robot.subsystems.intake.pivot.IntakePivotIOInputsAutoLogged;
import frc.robot.subsystems.intake.rollers.RollerIO;
import frc.robot.subsystems.intake.rollers.RollerIOInputsAutoLogged;

public class Intake extends ProfiledSubsystem {
    
    IntakePivotIO m_PivotIO;

    IntakePivotIOInputsAutoLogged m_pivotInputs;
    RollerIO m_rollerIO;
    RollerIOInputsAutoLogged m_rollerInputs;

    public Intake(IntakePivotIO io, RollerIO rollerIO) {
        super();
        m_PivotIO = io;
        m_rollerIO = rollerIO;
        m_pivotInputs = new IntakePivotIOInputsAutoLogged();
        m_rollerInputs = new RollerIOInputsAutoLogged();
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void periodic() {
        m_PivotIO.updateInputs(m_pivotInputs);
        Logger.processInputs("Intake Pivot", m_pivotInputs);
        m_rollerIO.updateInputs(m_rollerInputs);
        Logger.processInputs("Intake Roller", m_rollerInputs);
    }


    public void setPivotAngle(Rotation2d angle) {
        m_PivotIO.setDesiredAngle(angle);
    }

    // public void setIntakeSpeed(double speed) {
    //     m_rollerIO.setVoltage(speed);
    // }


    public Transform3d getTransform() {

        return new Transform3d(-0.515+.287, 0, 0.233, new Rotation3d(0, m_pivotInputs.curAngle, 0));
        // return new Transform3d(-0, 0, 0, new Rotation3d(0, m_rotation.getRadians(), 0));
    }
}
