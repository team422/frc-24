package ClimbTesting;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants.Vision.ClimbConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOSim;

public class ClimbTest {
    static final double DELTA = 0.02; // 2 cm error is probably fineee
    Climb m_climb;

    void sleep(int seconds) {
        sleepCycles(seconds * 50);
    }

    void sleepCycles(int cycles) {
        for (int i = 0; i < cycles; i++) {
            m_climb.periodic();
            // System.out.println("Height: " + m_climb.getHeight());
        }
    }

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
        m_climb = new Climb(new ClimbIOSim(),
                new ProfiledPIDController(ClimbConstants.kClimbP.get(), ClimbConstants.kClimbI.get(),
                        ClimbConstants.kClimbD.get(),
                        new Constraints(ClimbConstants.kMaxVelocity, ClimbConstants.kMaxAcceleration)),
                ClimbConstants.kMinHeight,
                ClimbConstants.kMaxHeight);
    }

    @Test
    void testHeights() {
        double maxHeight = ClimbConstants.kMaxHeight;
        double dHeight = 0.1;
        double height = ClimbConstants.kMinHeight;
        while (height < maxHeight) {
            m_climb.setDesiredHeight(height);
            // System.out.println("Init Height: " + m_climb.getHeight());
            // System.out.println("Desired Height: " + height);
            sleep(1);
            // System.out.println("Final Height: " + m_climb.getHeight());
            assert Math.abs(m_climb.getHeight() - height) < DELTA;
            height += dHeight;
        }
    }

    @Test
    void testMove() {
        while (m_climb.getHeight() < ClimbConstants.kMaxHeight) {
            double height = m_climb.getHeight();
            m_climb.setDesiredHeight(height + 0.1);
            // System.out.println("Init Height: " + m_climb.getHeight());
            // System.out.println("Desired Height: " + height);
            sleep(2);
            // System.out.println("Final Height: " + m_climb.getHeight() + "\n");
            assert Math.abs(m_climb.getHeight() - (height + 0.1)) < DELTA;
        }
    }

    @Test
    void testToZero() {
        m_climb.setDesiredHeight(ClimbConstants.kMaxHeight);
        // System.out.println("Init Height: " + m_climb.getHeight());
        // System.out.println("Desired Height: " + height);
        sleep(5);
        // System.out.println("Final Height: " + m_climb.getHeight() + "\n");
        m_climb.setDesiredHeight(0);
        // System.out.println("Init Height: " + m_climb.getHeight());
        // System.out.println("Desired Height: " + height);
        sleep(5);
        // System.out.println("Final Height: " + m_climb.getHeight() + "\n");
        assert Math.abs(m_climb.getHeight()) < DELTA;
    }
}
