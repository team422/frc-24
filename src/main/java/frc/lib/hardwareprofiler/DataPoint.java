package frc.lib.hardwareprofiler;

import edu.wpi.first.wpilibj.Timer;

public class DataPoint {
  public double time;
  public double[] values;

  public DataPoint(double time, double[] values) {
    this.time = time;
    this.values = values;
  }

  public DataPoint(double[] values) {
    this.time = Timer.getFPGATimestamp();
    this.values = values;
  }
}
