package frc.robot.subsystems.objectVision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ObjectDetectionIO {
     class ObjectDetectionVisionIOInputs implements LoggableInputs {
    public double[] timestamps = new double[] {};
    public double[][] objects = new double[][] {};
    public long fps = 0;

    @Override
    public void toLog(LogTable table) {
      table.put("Timestamps", timestamps);
      table.put("ObjectCount", objects.length);
      for (int i = 0; i < objects.length; i++) {
        table.put("Objects/" + i, objects[i]);
      }
      table.put("Fps", fps);
    }

    @Override
    public void fromLog(LogTable table) {
      timestamps = table.get("Timestamps", new double[] {0.0});
      int objectCount = table.get("objectCount", 0);
      objects = new double[objectCount][];
      for (int i = 0; i < objectCount; i++) {
        objects[i] = table.get("Frame/" + i, new double[] {});
      }
      fps = table.get("Fps", 0);
    }
  }

  default void updateInputs(ObjectDetectionVisionIOInputs inputs) {} 
}
