package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {

    public enum ClimbState {
        LOCKED,
        UNLOCKED
    }

    ClimbState m_climbState = ClimbState.LOCKED;

    ClimbIO m_climbIO;

    public Climb(ClimbIO climbIO) {
        m_climbIO = climbIO;
    }

    public void setServoPosition(double position) {
        m_climbIO.setServoPosition(position);
    }


    public void setSpeed(double speed) {
        m_climbIO.setSpeed(speed);
    }


    public void lockServo() {
        m_climbIO.lockServos();
        m_climbState = ClimbState.LOCKED;
    }

    public void unlockServo() {
        m_climbIO.unlockServos();
        m_climbState = ClimbState.UNLOCKED;
    }

    public void toggleServo() {
        if (m_climbState == ClimbState.LOCKED) {
            unlockServo();
        } else {
            lockServo();
        }
        
    }
}