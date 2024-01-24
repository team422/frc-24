package frc.lib.hardwareprofiler;

public class PowerConsumptionHelper {
  double m_totalPowerConsumption;
  double m_lastTime;
  double m_lastCurrent;
  double m_lastVoltage;

  public PowerConsumptionHelper() {
    m_totalPowerConsumption = 0;
    m_lastTime = 0;
    m_lastCurrent = 0;
    m_lastVoltage = 0;
  }

  public void update(double current, double voltage, double time) {
    if (m_lastTime == 0) {
      System.out.println("RESETTING");
      m_lastTime = time;
      m_lastCurrent = current;
      m_lastVoltage = voltage;
      return;
    }
    // System.out.println("Current: " + current + " Voltage: " + voltage + " Time: " + time);
    if (time - m_lastTime > 0.5) {
      m_lastTime = time;
      m_lastCurrent = current;
      m_lastVoltage = voltage;
      return;
    }
    m_totalPowerConsumption += (current * voltage) * (time - m_lastTime);
    m_lastTime = time;
    m_lastCurrent = current;
    m_lastVoltage = voltage;
  }

  public double getPowerUsage() {
    return m_totalPowerConsumption;
  }
}
