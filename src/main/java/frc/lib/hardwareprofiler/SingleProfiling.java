package frc.lib.hardwareprofiler;

public class SingleProfiling {
  public enum Intensity {
    Quick, RequiresGroundMovement, Long
  }

  public ProfiledSubsystem[] subsystemsInvolved;
  public Enum<?>[] subsystemsTestProfiles;
  public boolean[] allSubsystemsReady;
  public boolean[] allSubsystemsFinished;
  public boolean requireAllSubsystemsFinished;
  public Intensity intensity;

  public SingleProfiling(ProfiledSubsystem[] subsystemsInvolved, Enum<?>[] subsystemsTestProfiles,
      Intensity intensity, boolean requireAllSubsystemsFinished) {
    this.subsystemsInvolved = subsystemsInvolved;
    this.subsystemsTestProfiles = subsystemsTestProfiles;
    this.allSubsystemsReady = new boolean[subsystemsInvolved.length];
    this.allSubsystemsFinished = new boolean[subsystemsInvolved.length];
    this.intensity = intensity;
    this.requireAllSubsystemsFinished = requireAllSubsystemsFinished;
  }

  public boolean allSubsystemsReady() {
    for (boolean subsystemReady : allSubsystemsReady) {
      if (!subsystemReady) {
        return false;
      }
    }
    return true;
  }

  public boolean allSubsystemsFinished() {
    if (!requireAllSubsystemsFinished) {
      for (boolean subsystemFinished : allSubsystemsFinished) {
        if (subsystemFinished) {
          return true;
        }
      }
    }
    for (boolean subsystemFinished : allSubsystemsFinished) {
      if (!subsystemFinished) {
        return false;
      }
    }
    return true;
  }

  public boolean stopAllTests() {
    for (ProfiledSubsystem subsystem : subsystemsInvolved) {
      subsystem.stopTestProfile();
    }
    return true;
  }

  public void setSubsystemReady(boolean ready, Enum<?> profile) {
    for (int i = 0; i < subsystemsInvolved.length; i++) {
      if (subsystemsInvolved[i] instanceof ProfiledSubsystem) {
        if (subsystemsTestProfiles[i] == profile) {
          allSubsystemsReady[i] = ready;
        }
      }
    }
  }

  public void setSubsystemFinished(Enum<?> profile) {
    for (int i = 0; i < subsystemsInvolved.length; i++) {
      if (subsystemsInvolved[i] instanceof ProfiledSubsystem) {
        if (subsystemsTestProfiles[i] == profile) {
          allSubsystemsFinished[i] = true;
        }
      }
    }
  }

}
