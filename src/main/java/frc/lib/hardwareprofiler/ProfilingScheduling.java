package frc.lib.hardwareprofiler;

import java.util.ArrayList;

public class ProfilingScheduling {
  private static ProfilingScheduling instance;
  public SingleProfiling[] allTests;
  public ArrayList<SingleProfiling.Intensity> intensities;
  public int currentTestIndex = -1;
  public boolean allTestsFinished = false;
  public SingleProfiling currentTest;

  private ProfilingScheduling(SingleProfiling[] tests, SingleProfiling.Intensity[] intensitiesToRun) {
    allTests = tests;
    intensities = new ArrayList<SingleProfiling.Intensity>();
    for (SingleProfiling.Intensity intensity : intensitiesToRun) {
      intensities.add(intensity);
    }
  }

  public static ProfilingScheduling startInstance(SingleProfiling[] tests,
      SingleProfiling.Intensity[] intensitiesToRun) {
    if (instance == null) {
      instance = new ProfilingScheduling(tests, intensitiesToRun);
    }
    return instance;
  }

  public static ProfilingScheduling getInstance() {
    return instance;
  }

  public void startNextTest() {
    currentTestIndex++;
    if (currentTestIndex < allTests.length) {
      SingleProfiling currentTest = allTests[currentTestIndex];
      if (intensities.contains(currentTest.intensity)) {
        for (int i = 0; i < currentTest.subsystemsInvolved.length; i++) {
          if (currentTest.subsystemsInvolved[i] instanceof ProfiledSubsystem) {
            ((ProfiledSubsystem) currentTest.subsystemsInvolved[i])
                .setTestProfile(currentTest.subsystemsTestProfiles[i]);
          }
        }
        this.currentTest = currentTest;
      }
    }
  }

  public void readyNextPoint(boolean ready, Enum<?> profile) {
    if (currentTest != null) {
      currentTest.setSubsystemReady(ready, profile);
    }
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
      currentTest.setSubsystemFinished(profile);
      if (currentTest.allSubsystemsFinished()) {
        currentTest.stopAllTests();
        startNextTest();
      }
    }
  }

}
