package frc.robot.utils;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class LoggerWrapper {
  private static int cycle;

  public static void incrementCycle() {
    cycle++;
  }
  
  public static void recordOutput(String key, String value, int period) {
    if (cycle % period == 0) {
      Logger.recordOutput(key, value);
    }
  }

  public static void recordOutput(String key, long value, int period) {
    if (cycle % period == 0) {
      Logger.recordOutput(key, value);
    }
  }

  public static void recordOutput(String key, double value, int period) {
    if (cycle % period == 0) {
      Logger.recordOutput(key, value);
    }
  }

}
