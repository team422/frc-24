package frc.robot.subsystems.climb;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.ClimbConstants;

public class ClimbIOTalon implements ClimbIO {


    TalonFX leaderTalon;
    TalonFX followerTalon;
    Servo m_servo;
    Servo m_servo2;
    
    VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withUpdateFreqHz(0.0);



    StatusSignal<Double> curHeight;

    public ClimbIOTalon(int motorID1, int motorID2, int servoID1, int servoID2) {
        leaderTalon = new TalonFX(motorID1);
        followerTalon = new TalonFX(motorID2);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.RotorToSensorRatio = ClimbConstants.gearboxRatio;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimit = 40.0; 
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        leaderTalon.getConfigurator().apply(config);

        followerTalon.setControl(new Follower(motorID1, true));

        curHeight = leaderTalon.getPosition();
        
        m_servo = new Servo(servoID1);
        m_servo2 = new Servo(servoID2);
    }

    @Override
    public void lockServos() {
        m_servo.set(ClimbConstants.kLockPosition);
        m_servo2.set(ClimbConstants.kLockPosition);
    }

    @Override
    public void unlockServos() {
        m_servo.set(ClimbConstants.kUnlockPosition);
        m_servo2.set(ClimbConstants.kUnlockPosition);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.height = curHeight.getValueAsDouble();
    }
    
}
