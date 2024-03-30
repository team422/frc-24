package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.hardwareprofiler.ProfiledSubsystem;
import frc.lib.utils.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.intake.pivot.IntakePivotIO;
import frc.robot.subsystems.intake.pivot.IntakePivotIOInputsAutoLogged;
import frc.robot.subsystems.intake.rollers.RollerIO;
import frc.robot.subsystems.intake.rollers.RollerIOInputsAutoLogged;

public class Intake extends ProfiledSubsystem {
    
    IntakePivotIO m_PivotIO;

    IntakePivotIOInputsAutoLogged m_pivotInputs;
    RollerIO m_rollerIO;
    RollerIOInputsAutoLogged m_rollerInputs;

    Rotation2d m_rotation;

    public Intake(IntakePivotIO io, RollerIO rollerIO) {
        super();
        m_PivotIO = io;
        m_rollerIO = rollerIO;
        m_pivotInputs = new IntakePivotIOInputsAutoLogged();
        m_rollerInputs = new RollerIOInputsAutoLogged();
        m_rotation = io.getAngle();
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void periodic() {
        double start = HALUtil.getFPGATime();
        LoggedTunableNumber.ifChanged(
      hashCode(), () -> {
        m_PivotIO.setPID(IntakeConstants.kIntakeP.get(),IntakeConstants.kIntakeI.get(),IntakeConstants.kIntakeD.get());
        }, IntakeConstants.kIntakeP,IntakeConstants.kIntakeI,IntakeConstants.kIntakeD);
        m_PivotIO.updateInputs(m_pivotInputs);
        Logger.processInputs("Intake Pivot", m_pivotInputs);
        m_rollerIO.updateInputs(m_rollerInputs);
        Logger.processInputs("Intake Roller", m_rollerInputs);
        if (Constants.fullManualIntakePivotAndSpeedControls){
            m_PivotIO.setDesiredAngle(Rotation2d.fromDegrees(IntakeConstants.kIntakePivotManualControl.get()));
            m_rollerIO.setVoltage(IntakeConstants.kIntakeRollerManualControl.get());
        } else {
        if (RobotState.getInstance().getMaxIntakeAngle().getDegrees() < m_rotation.getDegrees()){
            m_PivotIO.setDesiredAngle(RobotState.getInstance().getMaxIntakeAngle());
        } else {
            m_PivotIO.setDesiredAngle(m_rotation);
        }
        }
        Logger.recordOutput("LoggedRobot/IntakePeriodic", (HALUtil.getFPGATime()-start)/1000);
    }

    public void setRollerVoltage(double voltage) {
        m_RollerIO.setVoltage(voltage);
    }

    public void setPivotAngle(Rotation2d angle) {
        m_rotation = angle;
        // m_PivotIO.setDesiredAngle(angle);
    }

    public void addDegreesToPivotAngle(double degrees) {
        m_rotation = m_rotation.plus(Rotation2d.fromDegrees(degrees));
        // m_PivotIO.setDesiredAngle(m_PivotIO.getAngle().plus(Rotation2d.fromDegrees(degrees)));
    }

    public void setIntakeSpeed(double speed) {
        m_rollerIO.setVoltage(speed);
    }


    public Transform3d getTransform() {

        return new Transform3d(-0.515+.287, 0, 0.233, new Rotation3d(0, m_pivotInputs.curAngle, 0));
        // return new Transform3d(-0, 0, 0, new Rotation3d(0, m_rotation.getRadians(), 0));
    }

    public Rotation2d getAngle() {
        return m_rotation;
    }
}
