package frc.robot.subsystems.amp;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.amp.AmpIO.AmpIOInputs;

public class AmpIOFalcon implements AmpIO{
    TalonFX m_motor;   
    TalonFXConfiguration m_config;
    SimpleMotorFeedforward mArmFeedforward;


    public AmpIOFalcon(int ampMotor){
        

        m_motor = new TalonFX(ampMotor);
        m_config  =  new TalonFXConfiguration();
        m_config.Feedback.SensorToMechanismRatio = 16.5;
        m_config.CurrentLimits.SupplyCurrentLimit = 5;

        m_config.Slot0.kP = 0.1;
        m_config.Slot0.kI = 0.0;
        m_config.Slot0.kD = 0.0;
        m_motor.setPosition(0);
        m_motor.getConfigurator().apply(m_config);
        mArmFeedforward = new SimpleMotorFeedforward(0.3, 0, 0);
        // m_config.ClosedLoopGeneral.ContinuousWrap = true;
        
        
    }


    public void setPosition(Rotation2d pos){
        
        m_motor.setControl(new PositionVoltage(pos.getRotations()).withSlot(0).withFeedForward(mArmFeedforward.calculate(pos.minus(getPosition()).getDegrees(),0)));

    }

    public void zero(){
        m_motor.setControl(new PositionVoltage(-.1).withSlot(0));
        m_motor.setPosition(0);
    }


    public void updateInputs(AmpIOInputs inputs){
        
        inputs.current = m_motor.getSupplyCurrent().getValueAsDouble();
        inputs.voltage = m_motor.getMotorVoltage().getValueAsDouble();
        inputs.angleDegrees = Rotation2d.fromRotations(m_motor.getPosition().getValueAsDouble()).getDegrees();
        inputs.velocity = m_motor.getVelocity().getValueAsDouble();
    }


    @Override
    public Rotation2d getPosition() {
        return Rotation2d.fromRotations(m_motor.getPosition().getValueAsDouble());
    }


    @Override
    public void setPID(double p, double i, double d) {
        m_config.Slot0.kP = p;
        m_config.Slot0.kI = i;
        m_config.Slot0.kD = d;

        m_motor.getConfigurator().apply(m_config);



    }
}
