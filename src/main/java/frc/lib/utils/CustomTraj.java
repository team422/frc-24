package frc.lib.utils;

import java.lang.reflect.Array;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;

public class CustomTraj {
    ArrayList<Pose2d> m_poses;
    ArrayList<Pose2d> m_posesToGo;
    ArrayList<Twist2d> m_velocities;
    ArrayList<Twist2d> m_velocitiesToGo;
    ArrayList<Twist2d> m_accelerations;
    ArrayList<Twist2d> m_accelerationsToGo;
    Double m_deltaTimes;
    int currIndex;

    public CustomTraj(ArrayList<Pose2d> poses, ArrayList<Twist2d> velocities, ArrayList<Twist2d> accelerations, Double deltaTimes) {
        m_poses = poses;
        m_velocities = velocities;
        m_accelerations = accelerations;
        m_deltaTimes = deltaTimes;
        currIndex = 0;
    }

    public void replacePath(ArrayList<Pose2d> poses, ArrayList<Twist2d> velocities, ArrayList<Twist2d> accelerations) {
        // m_poses = m_poses.subList(0, currIndex+1).addAll(poses);
        m_posesToGo = poses;
        m_velocitiesToGo = velocities;
        m_accelerationsToGo = accelerations;
    }

    public Pose2d getCurrPose() {
        return m_poses.get(currIndex);
    }

    public Twist2d getCurrVelocity() {
        return m_velocities.get(currIndex);
    }

    public Twist2d getCurrAcceleration() {
        return m_accelerations.get(currIndex);
    }

    public Double getCurrDeltaTime() {
        return m_deltaTimes;
    }

    public void update() {
        currIndex++;
    }

}