package frc.robot.subsystems.intake.rollers;


import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.units.Measure;
import frc.robot.Constants.IntakeConstants;

public class RollerIOKraken implements RollerIO {
    TalonFX m_motor;
    TalonFXConfigurator config;
    double m_speed;
    Supplier<Double> getSpeed;
    public RollerIOKraken(int motorID,Supplier<Double> m_Speed ) {
        m_motor = new TalonFX(motorID,"rio");
        config = m_motor.getConfigurator();
        getSpeed = m_Speed;
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = 40;
        currentLimits.SupplyTimeThreshold = 0.4;
        currentLimits.SupplyCurrentLimitEnable = true;
        currentLimits.SupplyCurrentThreshold = 50;
        config.apply(currentLimits);
        FeedbackConfigs feedback = new FeedbackConfigs();
        feedback.SensorToMechanismRatio = IntakeConstants.intakeSpeedToMPS;
        feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.curVelocity = getVelocity();
        inputs.voltage = m_motor.getMotorVoltage().getValueAsDouble();
        inputs.outputCurrent = m_motor.getSupplyCurrent().getValueAsDouble();
        inputs.desiredSpeeds = m_speed;
        System.out.println(getSpeed.get()*12);
        m_motor.setVoltage(-getSpeed.get()*12);
    }

    public double getVelocity(){
        return m_motor.getVelocity().getValueAsDouble();

    }

    @Override
    public void setVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }
    
    
}
