package frc.lib.utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.FieldConstants;

public class FieldUtil {
  public static final String kDefaultFieldName = "Field";
  private static Map<String, FieldUtil> fieldInstances = new HashMap<>();

  private final Field2d field = new Field2d();

  private FieldUtil(String name) {
    SmartDashboard.putData(name, field);
  }

  public static synchronized FieldUtil getDefaultField() {
    return getField(kDefaultFieldName);
  }

  public static synchronized FieldUtil getField(String name) {
    if (!fieldInstances.containsKey(name)) {
      fieldInstances.put(name, new FieldUtil(name));
    }

    return fieldInstances.get(name);
  }

  public void setObjects(Map<String, Pose2d> objects) {
    for (var entry : objects.entrySet()) {
      String objectName = entry.getKey();
      Pose2d objectPose = entry.getValue();
      setObjectPose(objectName, objectPose);
    }
  }

  public void setObjectGlobalPose(String name, Pose2d pose) {
    field.getObject(name).setPose(pose);
  }

  public void setObjectPose(String name, Pose2d pose) {
    setObjectGlobalPose(name, transformPose(pose));
  }

  public void setObjectGlobalPoses(String name, Pose2d... poses) {
    field.getObject(name).setPoses(poses);
  }

  public void setObjectPoses(String name, Pose2d... poses) {
    setObjectGlobalPoses(name, transformPoses(poses));
  }

  public void setTrajectory(String name, Trajectory trajectory) {
    field.getObject(name).setTrajectory(trajectory);
  }

  public void updateRobotPose(Pose2d pose) {
    field.setRobotPose(transformPose(pose));
  }

  public void setSwerveRobotPose(Pose2d pose, SwerveModuleState[] states, Translation2d[] translations) {
    updateRobotPose(pose);

    Pose2d[] modulePoses = new Pose2d[states.length];

    for (int i = 0; i < states.length; i++) {
      var translation = translations[i];
      var rotation = states[i].angle;

      modulePoses[i] = pose.transformBy(new Transform2d(translation, rotation));
    }

    setObjectPoses("RobotSwerveModules", modulePoses);
  }

  public Pose2d getObjectPose(String name) {
    return field.getObject(name).getPose();
  }

  public void removeObject(String name) {
    field.getObject(name).setPoses();
  }

  private Pose2d[] transformPoses(Pose2d[] poses) {
    Pose2d[] result = new Pose2d[poses.length];
    for (int i = 0; i < poses.length; i++) {
      result[i] = transformPose(poses[i]);
    }

    return result;
  }

  private Pose2d transformPose(Pose2d pose) {
    if(Robot.isSimulation()){
      if(DriverStation.getAlliance().isEmpty()){
        return pose;
      }
    }
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      return pose;
    }

    return pose.relativeTo(FieldConstants.kOppositeField);
  }

}
