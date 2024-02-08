package IntakeTesting;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.pivot.PivotIOSim;
import frc.robot.subsystems.intake.rollers.RollerIOSim;

public class IntakeTest {
    static final double DELTA = 0.2;
    Intake m_intake;

    void sleep(int seconds) {
        sleepCycles(seconds * 50);
    }

    void sleepCycles(int cycles) {
        for (int i = 0; i < cycles; i++) {
            m_intake.periodic();
        }
    }

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
        m_intake = new Intake(new PivotIOSim(), new RollerIOSim(), 12);
    }

    @Test
    void testRollerVoltages() {
        double[] voltages = {-12, -9, -6, -3, -1, -0.5, 0, 0.5, 1, 3, 6, 9, 12};
        m_intake.setRollerVoltage(voltages[0]);
        sleep(1);
        double prevVoltage = m_intake.m_rollerInputs.voltage;
        for (int i = 1; i < voltages.length; i++) {
            m_intake.setRollerVoltage(voltages[i]);
            sleep(1);
            assertTrue(m_intake.m_rollerInputs.voltage > prevVoltage - DELTA);
            prevVoltage = m_intake.m_rollerInputs.voltage;

            m_intake.setRollerVoltage(0);
            sleep(2);
        }
    }

}
