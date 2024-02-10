package frc.robot.subsystems.shooter.flywheel;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;


public class FlywheelIOSim implements FlywheelIO {
    
    private FlywheelSim m_flywheel;
    public final double m_wheelLength;
    //again i just in random numbers, i dunno what to put
    public FlywheelIOSim() {
        DCMotor gearbox = DCMotor.getFalcon500(1);
        double gearing = 3;
        double jKgMetersSquared = 0.1;
        m_flywheel = new FlywheelSim(gearbox, gearing, jKgMetersSquared);
        m_wheelLength = Units.inchesToMeters(1);
      }

      public void updateInputs(FlywheelInputsAutoLogged inputs) {
        m_flywheel.update(.02);
        inputs.velocityMetersPerSec = getVelocityMetersPerSec();
        inputs.velocityRadPerSec = getVelocityRadPerSec();
      }

      @Override
      public void setVoltage(double voltage) {
        m_flywheel.setInputVoltage(voltage);
      }

      @Override
      public double getVelocityRadPerSec() {
        return m_flywheel.getAngularVelocityRadPerSec();
      }

      @Override
      public double getVelocityMetersPerSec() {
        return m_flywheel.getAngularVelocityRadPerSec() * m_wheelLength;
      }
    
      @Override
      public double getVelocityRevPerMin() {
        return m_flywheel.getAngularVelocityRPM();
      }
    
      public double getWheelLength() {
        return m_wheelLength;
      }


}
