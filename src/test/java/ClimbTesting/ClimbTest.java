package ClimbTesting;

import static org.junit.jupiter.api.Assertions.assertEquals;

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
    /*
     * TO-DO: redo tests
     * a manual input test is best done by using
     * 1 second of one direction
     * 1 second of wait
     * 1 second of the other direction
     * and one second of wait
     * and start again
     * if you can do that twice you are probably golden
     * each time, check that the rate of change has the right sign
     */

    @Test
    void testHeights() {
        double maxHeight = ClimbConstants.kMaxHeight;
        double dHeight = 0.1;
        double height = ClimbConstants.kMinHeight;
        while (height < maxHeight) {
            m_climb.setDesiredHeight(height);
            sleep(1);
            assertEquals(m_climb.getHeight(), height, DELTA);
            height += dHeight;
        }
    }

    @Test
    void testMove() {
        double[] times = new double[5];
        double speed = ClimbConstants.kClimbUpSpeed.get();
        for (int i = 0; i < 5; i++) {
            times[i] = i * 0.5;
        }
        for (double time : times) {
            m_climb.setDesiredHeight(ClimbConstants.kMinHeight + 0.5);
            sleep(5);
            m_climb.moveCommand(speed).execute();
            sleepCycles((int) (time * 50));
            m_climb.moveCommand(-speed).execute();
            sleepCycles((int) (time * 50));
            // System.out.println("Height: " + m_climb.getHeight() + " " + "Time: " + time + "s");
            assertEquals(m_climb.getHeight(), ClimbConstants.kMinHeight + 0.5, DELTA);
        }
    }

    @Test
    void testToZero() {
        m_climb.setDesiredHeight(ClimbConstants.kMaxHeight);
        // System.out.println("Init Height: " + m_climb.getHeight());
        // System.out.println("Desired Height: " + ClimbConstants.kMaxHeight);
        sleep(5);
        // System.out.println("Final Height: " + m_climb.getHeight() + "\n");
        m_climb.setDesiredHeight(0);
        // System.out.println("Init Height: " + m_climb.getHeight());
        // System.out.println("Desired Height: 0");
        sleep(5);
        // System.out.println("Final Height: " + m_climb.getHeight() + "\n");
        assert Math.abs(m_climb.getHeight()) < DELTA;
    }
}
