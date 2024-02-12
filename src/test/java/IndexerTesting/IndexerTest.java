package IndexerTesting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.sensor.SensorIO;
import frc.robot.subsystems.indexer.wheel.WheelIO;
import frc.robot.subsystems.indexer.wheel.WheelIOSim;
import frc.robot.subsystems.indexer.sensor.SensorIOSim;

public class IndexerTest {
    Indexer m_indexer;
    final double DELTA = 0.2;

    void sleep(int seconds) {
        sleepCycles(seconds * 50);
    }

    void sleepCycles(int cycles) {
        for (int i = 0; i < cycles; i++) {
            m_indexer.periodic();
        }
    }

    @BeforeEach
    public void setup() {
        assert HAL.initialize(500, 0);

        WheelIO wheelIO = new WheelIOSim();
        SensorIO sensorIOUp = new SensorIOSim();
        SensorIO sensorIODown = new SensorIOSim();
        ProfiledPIDController controller = new ProfiledPIDController(10, 0.05, 1,
            new Constraints(15, 20));
        controller.setTolerance(DELTA);
        m_indexer = new Indexer(wheelIO, sensorIOUp, sensorIODown, controller, DELTA);
    }

    @Test
    public void pidTest() {
        double[] setpoints = new double[41];
        // generate setpoints from -4pi to 4pi
        for (int i = 0; i < setpoints.length; i++) {
            setpoints[i] = -4 * Math.PI + i * (8 * Math.PI / (setpoints.length - 1));
        }
        for (double setpoint : setpoints) {
            m_indexer.setSetpoint(setpoint);
            sleep(1);
            System.out.println("Setpoint: " + setpoint + " Position: " + m_indexer.m_wheelInputs.position);
            // assertEquals(m_indexer.m_wheelInputs.position, setpoint, DELTA);
        }
    }
}
