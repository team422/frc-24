package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
    Servo m_servo;
    Servo m_servo2;

    public Climb() {
        m_servo = new Servo(ClimbConstants.kServoPort);
        m_servo2 = new Servo(ClimbConstants.kServoPort2);
    }

    public void setServoPosition(double position) {
        m_servo.set(position);
    }



    public void deltaServoPosition(double position){
        System.out.println(m_servo.get());
        System.out.println(m_servo.get()+position);
        m_servo.set(m_servo.get()+position);
    }


    public void lockServo() {
        m_servo.set(ClimbConstants.kLockPosition);
        m_servo2.set(ClimbConstants.kLockPosition);
    }

    public void unlockServo() {
        m_servo.set(ClimbConstants.kUnlockPosition);
        m_servo2.set(ClimbConstants.kUnlockPosition);

    }
}