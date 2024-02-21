package frc.lib.hardwareprofiler;

import java.util.ArrayList;

public class ProfilingScheduling {
  private static ProfilingScheduling instance;
  public ArrayList<SingleProfiling> allTests;
  public ArrayList<SingleProfiling.Intensity> intensities;
  public boolean allTestsFinished = false;
  public SingleProfiling currentTest;

  private ProfilingScheduling() {
    allTests = new ArrayList<SingleProfiling>();
    intensities = new ArrayList<SingleProfiling.Intensity>();
  }

  public void addTest(SingleProfiling test, SingleProfiling.Intensity intensity) {
    allTests.add(test);
    intensities.add(intensity);
  }

  public static void updateCurrent() {
    instance.getCurrentTest().update();
  }

  public SingleProfiling getCurrentTest() {
    if (currentTest == null) {
      startNextTest();
    }
    return currentTest;
  }

  public static boolean hasTests() {
    return instance.allTests.size() > 0;
  }

  public static ProfilingScheduling startInstance() {
    if (instance == null) {
      instance = new ProfilingScheduling();
    }
    return instance;
  }

  public static ProfilingScheduling getInstance() {
    return instance;
  }

  public void startNextTest() {
    SingleProfiling currentTest = allTests.get(0);
    if (intensities.contains(currentTest.intensity)) {
      for (int i = 0; i < currentTest.subsystemsInvolved.length; i++) {
        currentTest.isResetting = true;
      }
      this.currentTest = currentTest;
    }
  }

  public void readyNextPoint(boolean ready, Enum<?> profile) {
  }

  public boolean checkReadyNextPoint() {
    if (currentTest != null) {
      if (currentTest.allSubsystemsReady()) {
        return true;
      }
    }
    return false;
  }

  public void setFinishTest(Enum<?> profile) {
    if (currentTest != null) {
      allTests.remove(0);
      currentTest.setSubsystemFinished(profile);
      currentTest = null;
      if (currentTest.allSubsystemsFinished()) {
        currentTest.resetToPose();
        startNextTest();
      }
    }
  }

}
