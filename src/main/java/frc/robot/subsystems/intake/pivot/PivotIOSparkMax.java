package frc.robot.subsystems.intake.pivot;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.advantagekit.LoggedIO;
import frc.lib.utils.CanSparkMaxSetup;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.pivot.IntakePivotIO.IntakePivotIOInputs;

public class PivotIOSparkMax implements IntakePivotIO {

    CANSparkMax m_motor;
    AbsoluteEncoder m_encoder;
    Rotation2d m_desiredAngle;
    SparkPIDController m_controller;

    

    public PivotIOSparkMax(int motorID) {
        m_motor = new CANSparkMax(motorID, MotorType.kBrushless);
        m_encoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);
        m_encoder.setPositionConversionFactor(Constants.IntakeConstants.pivotGearRatio);
        m_encoder.setZeroOffset(Rotation2d.fromDegrees(78).getRadians());
        m_motor.setIdleMode(IdleMode.kBrake);

        m_desiredAngle = Rotation2d.fromRadians(m_encoder.getPosition());

        m_motor.setSmartCurrentLimit(45);
        
        m_controller = m_motor.getPIDController();
        m_controller.setFeedbackDevice(m_encoder);
        m_controller.setP(IntakeConstants.kIntakeP.get(), 0);
        m_controller.setI(IntakeConstants.kIntakeI.get(), 0);
        m_controller.setD(IntakeConstants.kIntakeD.get(), 0);

        m_controller.setIMaxAccum(10, 0);


    }


    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {

        inputs.curAngle = Rotation2d.fromRadians(m_encoder.getPosition()).getDegrees();

        inputs.curSpeed = m_motor.getEncoder().getVelocity();
        inputs.desiredAngle = m_encoder.getPosition();
        inputs.voltage = m_motor.getAppliedOutput();
        inputs.desiredAngle = m_desiredAngle.getDegrees();
        inputs.desiredVoltage = m_motor.get();
        if(inputs.curAngle < 0 && inputs.desiredAngle < 0 ){
            m_motor.setSecondaryCurrentLimit(10);
        }else {
            m_motor.setSecondaryCurrentLimit(80);
        }

    }

    @Override
    public void setDesiredAngle(Rotation2d angle) {
        m_desiredAngle = angle;
        m_controller.setReference(angle.getRadians(), ControlType.kPosition, 0);
    }


    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(m_encoder.getPosition());
    }

    @Override
    public void setPID(double kP,double kI, double kD){
        m_controller.setP(kP, 0);
        m_controller.setI(kI, 0);
        m_controller.setD(kD, 0);
    }
   
    
}
