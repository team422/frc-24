import frc.robot.subsystems.shooter.pivot.PivotIO;
import frc.robot.subsystems.shooter.pivot.PivotIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.controller.ProfiledPIDController;

public class ShooterTest {
    Shooter m_shooter;
    final double DELTA = 0.3;

    void sleepCycles(int cycles){
        for (int i = 0; i < cycles; i++){
            m_shooter.periodic();
            m_shooter.simulationPeriodic();
        }
    }

    void sleep(double seconds){
        int cycles = (int) (seconds * 50);
        sleepCycles(cycles);
    }

    @BeforeEach
    void setup(){
        PivotIO pivotIO = new PivotIOSim();
        ProfiledPIDController controller = new ProfiledPIDController(
            40, 0.1, 0.1,
            new Constraints(20, 20)
        );
        FlywheelIO flywheelIO = new FlywheelIOSim();
        double tolerance = DELTA;
        m_shooter = new Shooter(pivotIO, controller, flywheelIO, tolerance);
    }

    @Test
    void velocityTest(){
        double[] velocities = new double[143];
        // 14.22 m/s is max
        for (int i = 0; i < velocities.length; i++){
            velocities[i] = i / 10.0;
        }
        for (double velocity : velocities){
            m_shooter.setVelocityMetersPerSecond(velocity * 0.1);
            sleep(5);

            m_shooter.setVelocityMetersPerSecond(velocity); 
            sleep(1.56);

            assertEquals(velocity, m_shooter.m_flywheelInputs.velocityMetersPerSec, DELTA);
        }
    }


}