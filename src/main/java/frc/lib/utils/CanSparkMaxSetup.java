package frc.lib.utils;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

public class CanSparkMaxSetup {
  public void setupSparkMaxSlow(CANSparkMax sparkMax) {
    // sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    // sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);
  }

  public void setupSparkMaxSlowFully(CANSparkMax sparkMax) {
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);
  }
}
