package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    Servo m_servo;
    public Climb() {
        m_servo = new Servo(9);
    }

    public void setServoPosition(double position) {
        m_servo.set(position);
    }

    public void deltaServoPosition(double position){
        System.out.println(m_servo.get());
        System.out.println(m_servo.get()+position);
        m_servo.set(m_servo.get()+position);
    }

}
