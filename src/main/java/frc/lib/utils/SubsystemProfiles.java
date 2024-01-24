package frc.lib.utils;

import java.util.HashMap;

public class SubsystemProfiles {
  public Object[] profileInstances;
  public Enum<?> currentProfile;
  public HashMap<Enum<?>, Runnable> profilePeriodicFunctions;
  public Enum<?> lastProfile;

  public SubsystemProfiles(Class<? extends Enum<?>> ProfileEnum, HashMap<Enum<?>, Runnable> profilePeriodicFunctions,
      Enum<?> defaultProfile) {
    profileInstances = ProfileEnum.getEnumConstants();
    currentProfile = defaultProfile;
    this.profilePeriodicFunctions = profilePeriodicFunctions;
    lastProfile = currentProfile;
  }

  public Runnable getPeriodicFunction() {
    return profilePeriodicFunctions.get(currentProfile);
  }

  public void setCurrentProfile(Enum<?> profile) {
    lastProfile = currentProfile;
    currentProfile = profile;
  }

  public void revertToLastProfile() {
    setCurrentProfile(lastProfile);
  }
}
