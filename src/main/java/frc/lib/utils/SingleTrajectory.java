package frc.lib.utils;
import java.util.ArrayList;

import edu.wpi.first.math.proto.Trajectory;

public class SingleTrajectory {
    public String name;
    public Trajectory trajectory;
    public ArrayList<Integer> keyPoints;
    public ArrayList<String> keyPointNames;
    public TrajGenerator trajGenerator;
    public CreationTime creationTime;
    

    public enum TrajGenerator {
        kTEB,
        kPathPlanner,
        kChoreo
    }

    public enum CreationTime {
        kPreMatch,
        kRuntime
    }

    public SingleTrajectory() {}

    public void setTrajectoryToPlanFor(String type){
        // set in network tables the type of trajectory to plan for
    }


}