package frc.lib.hardwareprofiler;

import java.io.File;
import java.io.FileWriter;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.UUID;

import com.google.gson.Gson;

import frc.robot.Robot;

public class HardwareProfiler {
  public String name;
  public double startTime;
  public int id;
  public int subsystemId;
  public int testNumber;
  public ArrayList<DataPoint> dataPoints;
  public String[] units;
  public ProfilingType profilingType;
  public Double[] testParamters;
  public String[] testParamtersName;
  public String time;

  public enum ProfilingType {
    POWER_CONSUMPTION, TIME_TO_SETPOINT, POWER_CONSUMPTION_BY_SETPOINT, DELTA_AT_TIME, CURRENT, VOLTAGE, TEMPERATURE,
    OTHER
  }

  public HardwareProfiler(String name, double time, int id, int subsystemId, String[] units,
      ArrayList<DataPoint> dataPoints,
      ProfilingType profilingType, int testNumber, Double[] testParamters, String[] testParamtersName) {
    this.name = name;
    this.startTime = time;
    this.id = id;
    this.subsystemId = subsystemId;
    this.dataPoints = dataPoints;
    this.units = units;
    this.profilingType = profilingType;
    this.testNumber = testNumber;
    this.testParamters = testParamters;
    this.testParamtersName = testParamtersName;
  }

  public HardwareProfiler(String name, double time, int id, int subsystemId, String[] units,
      ProfilingType profilingType, int testNumber, Double[] testParamters, String[] testParamtersName) {
    this.name = name;
    this.startTime = time;
    this.id = id;
    this.subsystemId = subsystemId;
    this.dataPoints = new ArrayList<DataPoint>();
    this.units = units;
    this.profilingType = profilingType;
    this.testNumber = testNumber;
    this.testParamters = testParamters;
    this.testParamtersName = testParamtersName;
  }

  public void addDataPoint(DataPoint dataPoint) {
    // DataPoint[] newDataPoints = new DataPoint[dataPoints.length + 1];
    // for (int i = 0; i < dataPoints.length; i++) {
    //   newDataPoints[i] = dataPoints[i];
    // }
    // newDataPoints[dataPoints.length] = dataPoint;
    // dataPoints = newDataPoints;
    dataPoints.add(dataPoint);
  }

  public void EnsureHardwareProfilerFolder() {
    if (Robot.isReal()) {
      File file = new File("/home/lvuser/hardwareprofiler");
      if (!file.exists()) {
        file.mkdir();
      }
    }
  }

  public void toJSON() {
    Gson gson = new Gson();
    time = LocalDateTime.now().toString();
    String json = gson.toJson(this);
    System.out.println(json);
    String fileName = time + "_" + id + "_" + subsystemId + "_" + name + "_" + UUID.randomUUID() + ".json";
    if (Robot.isReal()) {
      EnsureHardwareProfilerFolder();
      try {
        FileWriter fileWriter = new FileWriter("/home/lvuser/hardwareprofiler/" + fileName);
        fileWriter.write(json);
        fileWriter.close();
      } catch (Exception e) {
        System.out.println("Error writing to file");
      }
    } else {
      try {
        FileWriter fileWriter = new FileWriter("profilingData/" + fileName);
        fileWriter.write(json);
        fileWriter.close();
      } catch (Exception e) {
        try {
          // try on macos
          FileWriter fileWriter = new FileWriter("~/hardwareprofiler/" + fileName);
          fileWriter.write(json);
          fileWriter.close();
        } catch (Exception e2) {
          System.out.println("Error writing to file");
        }
      }
    }
  }

}
