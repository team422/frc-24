// Source code is decompiled from a .class file using FernFlower decompiler.

package frc.lib.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

public class LoggedDashboardNumber implements LoggedDashboardInput {
   private final String key;
   private double defaultValue;
   private double value;
   // private final LoggableInputs inputs;

   public LoggedDashboardNumber(String key) {
      this(key, 0.0);
   }

   public LoggedDashboardNumber(String key, double defaultValue) {
      // this.inputs = new 1(this);
      this.key = key;
      this.defaultValue = defaultValue;
      this.value = defaultValue;
      if (Constants.isTunableNetwork) {
         SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
      }
      // SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
      this.periodic();
      // Logger.registerDashboardInput(this);
   }

   public void setDefault(double defaultValue) {
      this.defaultValue = defaultValue;
   }

   public void set(double value) {
      // SmartDashboard.putNumber(this.key, value);
      if (Constants.isTunableNetwork) {
         SmartDashboard.putNumber(this.key, value);
      } else {
         this.value = value;
      }
   }

   public double get() {
      return this.value;
   }

   public void periodic() {
      if (!Logger.hasReplaySource()) {
         if (Constants.isTunableNetwork){
            this.value = SmartDashboard.getNumber(this.key, this.defaultValue);
         } else {
            this.value = this.defaultValue;
         }
      }

      // Logger.processInputs("DashboardInputs", this.inputs);
   }
}
