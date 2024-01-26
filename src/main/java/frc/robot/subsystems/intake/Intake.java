package frc.robot.subsystems.intake;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.hardwareprofiler.ProfiledSubsystem;
import frc.robot.subsystems.intake.pivot.PivotIO;
import frc.robot.subsystems.intake.pivot.PivotIOInputsAutoLogged;
import frc.robot.subsystems.intake.rollers.RollerIO;
import frc.robot.subsystems.intake.rollers.RollerInputsAutoLogged;

public class Intake extends ProfiledSubsystem {
    
    PivotIO m_PivotIO;
    RollerIO m_RollerIO;

    PivotIOInputsAutoLogged m_pivotInputs;
    RollerInputsAutoLogged m_rollerInputs;

    private final double m_intakeVoltage;

    public Intake(PivotIO io, RollerIO rollerIO, double intakeVoltage) {
        super();
        m_PivotIO = io;
        m_RollerIO = rollerIO;
        m_pivotInputs = new PivotIOInputsAutoLogged();
        m_rollerInputs = new RollerInputsAutoLogged();
        m_intakeVoltage = intakeVoltage;
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void periodic() {
        m_PivotIO.updateInputs(m_pivotInputs);
        m_RollerIO.updateInputs(m_rollerInputs);

        Logger.processInputs("Intake/Pivot", m_pivotInputs);
        Logger.processInputs("Intake/Roller", m_rollerInputs);

    }

    public void setRollerVoltage(double voltage) {
        m_RollerIO.setVoltage(voltage);
    }

    public void setPivotAngle(Rotation2d angle) {
        m_PivotIO.setDesiredAngle(angle);
    }
 
    public Transform3d getTransform() {
        return new Transform3d(-0.515+.287, 0, 0.233, new Rotation3d(0, m_pivotInputs.curAngle, 0));
        // return new Transform3d(-0, 0, 0, new Rotation3d(0, m_rotation.getRadians(), 0));
    }

    public Command startIntakeAtVoltage(double voltage) {
        return runEnd(
            () -> setRollerVoltage(voltage),
            () -> setRollerVoltage(0)
        );
    }

    public Command intakeCommand() {
        return startIntakeAtVoltage(m_intakeVoltage);
    }
}
