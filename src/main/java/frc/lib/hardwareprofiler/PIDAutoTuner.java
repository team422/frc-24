package frc.lib.hardwareprofiler;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;

public class PIDAutoTuner {
  // This uses the Ziegler-Nichols method to tune a PID controller
  // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
  double m_ku;
  String m_name;
  double m_lastTime;

  ArrayList<Double> m_max = new ArrayList<Double>();
  ArrayList<Double> m_min = new ArrayList<Double>(); // min and max values for each axis
  ArrayList<Double> m_times = new ArrayList<Double>(); // time between peaks

  double m_kp;

  ArrayList<Double> m_dataPoints;

  double m_setpoint;

  ArrayList<Double> m_kpValues = new ArrayList<Double>();
  ArrayList<Double> m_kMaxSlopeValues = new ArrayList<Double>();
  ArrayList<Double> m_kMinSlopeValues = new ArrayList<Double>();
  // start by increasing by a factor of 10
  // then use binary search to find the best value
  double m_kpStep = 2;

  boolean m_rising = true;
  boolean m_logarithmic = true;

  double m_NextLogTime = 0;

  double m_minChange;

  public PIDAutoTuner(String name, double setpoint, double minChange) {
    m_ku = 0;
    m_name = name;
    m_kp = 0.01;
    m_setpoint = setpoint;
    m_dataPoints = new ArrayList<Double>();
    m_minChange = minChange;
  }

  public void stampTime() {
    if (m_lastTime == 0) {
      m_lastTime = Timer.getFPGATimestamp();
      return;
    }
    m_dataPoints.add(Timer.getFPGATimestamp() - m_lastTime);
    m_lastTime = Timer.getFPGATimestamp();

  }

  public void checkExtrema(double val) {
    if (m_dataPoints.size() <= 10) {
      return;
    }
    ArrayList<Double> lastTen = new ArrayList<Double>();
    for (int i = m_dataPoints.size() - 10; i < m_dataPoints.size(); i++) {
      lastTen.add(m_dataPoints.get(i));
    }
    ArrayList<Double> lastHundred = new ArrayList<Double>();
    if (m_dataPoints.size() < 100) {
      // just use all the data points
      lastHundred = m_dataPoints;
    } else {
      for (int i = m_dataPoints.size() - 100; i < m_dataPoints.size(); i++) {
        lastHundred.add(m_dataPoints.get(i));
      }
    }
    if (Math.abs(calculateSlope(lastHundred)) < m_minChange && m_dataPoints.size() > 100) {
      calculateKu();
      return;
    }
    if (calculateSlope(lastTen) > 0) {
      m_rising = true;
    } else {
      m_rising = false;
    }
    if (m_rising && val < m_dataPoints.get(m_dataPoints.size() - 1)) {
      m_rising = false;
      // if (m_dataPoints.get(m_dataPoints.size() - 1) < m_setpoint) {
      m_max.add(m_dataPoints.get(m_dataPoints.size() - 1));
      // }
      stampTime();
    } else if (!m_rising && val > m_dataPoints.get(m_dataPoints.size() - 1)) {
      m_rising = true;
      m_min.add(m_dataPoints.get(m_dataPoints.size() - 1));
      stampTime();
    }

  }

  public boolean checkCriticalPoint(ArrayList<Double> values) {
    if (values.size() < 10) {
      return false;
    }

    double value = values.get(values.size() - 1);
    double lastValue = values.get(values.size() - 2);
    double secondLastValue = values.get(values.size() - 3);
    System.out.println(value < lastValue);
    System.out.println(lastValue > secondLastValue);
    if (value > lastValue && lastValue < secondLastValue) {
      return true;
    } else if (value < lastValue && lastValue > secondLastValue) {
      return true;
    }
    return false;
  }

  public boolean checkSignChange(ArrayList<Double> values) {
    if (values.size() < 3) {
      return false;
    }

    double value = values.get(values.size() - 1);
    double lastValue = values.get(values.size() - 2);
    if (value > 0 && lastValue < 0) {
      return true;
    } else if (value < 0 && lastValue > 0) {
      return true;
    }
    return false;
  }

  public void saveCurrentValue(double val) {
    if (Timer.getFPGATimestamp() < m_NextLogTime) {
      return;
    }
    checkExtrema(val);
    m_dataPoints.add(val);
  }

  public double getKp() {
    return m_kp;
  }

  public double getSetpoint() {
    return m_setpoint;
  }

  public void calculateKu() {
    // calculate scatter min slope by taking each point and plotting them 1 y away from each other
    // time should have a slope of close to 0 over time
    // calculate scatter max slope by taking each point and plotting them 1 y away from each other
    if (m_max.size() < 2 || m_min.size() < 2) {
      calculateNextKp();
      return;
    }
    double minSlope = calculateSlope(m_min);
    double maxSlope = calculateSlope(m_max);
    Logger.getInstance().recordOutput("maxSlope", maxSlope);
    Logger.getInstance().recordOutput("minSlope", minSlope);
    System.out.println(maxSlope);
    m_kpValues.add(m_kp);
    m_kMinSlopeValues.add(minSlope);
    m_kMaxSlopeValues.add(maxSlope);
    calculateNextKp();
    m_NextLogTime = Timer.getFPGATimestamp() + 5;
  }

  public void calculateNextKp() {
    if (m_kMaxSlopeValues.size() == 0) {
      m_kp *= m_kpStep;
      m_dataPoints.clear();
      m_max.clear();
      m_min.clear();
      return;
    }
    if (checkCriticalPoint(m_kMaxSlopeValues) && m_logarithmic) {
      m_kpStep = m_kp / 10;
      m_kp -= m_kpStep;
      m_kpStep /= 2;
      this.m_logarithmic = false;
    } else if (!m_logarithmic) {
      if (m_kMaxSlopeValues.get(m_kMaxSlopeValues.size() - 1) > 0) {
        m_kp += m_kpStep;
      } else {
        m_kp -= m_kpStep;
      }
      m_kpStep /= 2;
    } else if (m_logarithmic) {
      m_kp *= m_kpStep;
    }
    m_dataPoints.clear();
    m_max.clear();
    m_min.clear();

    System.out.println("Kp: " + m_kp);

  }

  public static double calculateSlope(ArrayList<Double> numbers) {
    if (numbers.size() < 2) {
      throw new IllegalArgumentException("At least 2 numbers are required to calculate the slope.");
    }

    double sumX = 0.0;
    double sumY = 0.0;
    double sumXY = 0.0;
    double sumX2 = 0.0;

    for (int i = 0; i < numbers.size(); i++) {
      double x = i;
      double y = numbers.get(i);

      sumX += x;
      sumY += y;
      sumXY += x * y;
      sumX2 += x * x;
    }

    double n = numbers.size();
    double slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
    return slope;
  }

}
