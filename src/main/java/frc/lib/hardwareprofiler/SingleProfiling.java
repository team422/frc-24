package frc.lib.hardwareprofiler;

import java.util.ArrayList;
import java.util.function.Consumer;

public class SingleProfiling {
  public enum Intensity {
    Quick, RequiresGroundMovement, Long
  }

  public ProfiledSubsystem[] subsystemsInvolved;
  public SubsystemInterfacer[] subsystemInterfacers;
  public Enum<?>[] subsystemsTestProfiles;
  public boolean[] allSubsystemsReady;
  public boolean[] allSubsystemsFinished;
  public boolean requireAllSubsystemsFinished;
  public Intensity intensity;
  public ArrayList<?> startPositions;
  public Consumer<?>[] set;

  public int testNumber;

  public ArrayList<?>[] testPositions;

  public enum PoseOrVelocity{
    Pose, Velocity
  };

  public boolean isResetting;

  public SingleProfiling(ProfiledSubsystem[] subsystemsInvolved, Enum<?>[] subsystemsTestProfiles,
      Intensity intensity, boolean requireAllSubsystemsFinished, SubsystemInterfacer[] subsystemInterfacers, ArrayList<?> startPositions,ArrayList<?>[] testPositions, PoseOrVelocity poseOrVelocity) {
    this.subsystemsInvolved = subsystemsInvolved;
    this.subsystemsTestProfiles = subsystemsTestProfiles;
    this.subsystemInterfacers = subsystemInterfacers;
    this.allSubsystemsReady = new boolean[subsystemsInvolved.length];
    this.allSubsystemsFinished = new boolean[subsystemsInvolved.length];
    this.intensity = intensity;
    this.requireAllSubsystemsFinished = requireAllSubsystemsFinished;
    this.testPositions = testPositions;

    testNumber = 0;

    this.set = new Consumer<?>[subsystemsInvolved.length];
    for (int i = 0; i < subsystemsInvolved.length; i++) {
      if(poseOrVelocity==PoseOrVelocity.Pose){
        this.startPositions = startPositions;
        this.set[i] = subsystemInterfacers[i].setPose;
      }
      else{
        this.startPositions = startPositions;
        this.set[i] = subsystemInterfacers[i].setVelocity;
      }
    }
  }

  public boolean allSubsystemsReady() {
    for (boolean subsystemReady : allSubsystemsReady) {
      if (!subsystemReady) {
        return false;
      }
    }
    return true;
  }

  @SuppressWarnings({"unchecked","rawtypes"})  
  public void resetToPose() {
    for (int i = 0; i < subsystemsInvolved.length; i++) {
        acceptHelper((Consumer)set[i],startPositions.get(i));
    }
  }

  @SuppressWarnings({"unchecked","rawtypes"})
  public void setToPoint() {
    for (int i = 0; i < subsystemsInvolved.length; i++) {
      acceptHelper((Consumer)set[i],testPositions[i].get(testNumber));
    }
  }

  private static <T> void acceptHelper(Consumer<T> consumer, T t) {
    // i hate templates
    consumer.accept(t);
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

  // public boolean stopAllTests() {
  //   for (ProfiledSubsystem subsystem : subsystemsInvolved) {
  //     subsystem.stopTestProfile();
  //   }
  //   return true;
  // }

  public void update() {
    for (int i = 0; i < subsystemsInvolved.length; i++) {
      if (isResetting){
        resetToPose();
      }else{
        setToPoint();

      }
    
    }
  }

  public void setSubsystemFinished(Enum<?> profile) {
    for (int i = 0; i < subsystemsInvolved.length; i++) {
      subsystemsInvolved[i].revertProfile();
    }
  }

}
