package frc.lib.utils;

import java.io.IOException;
import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.core.JacksonException;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.deser.std.StdDeserializer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import frc.robot.Constants;
import frc.robot.Constants.TrajectoryGenerationManager;

public class TrajectoryManager {
    // this should be able to deserialize a trajectory
    // keep and return a list of keypoints as well as if the robot is currently at them based on deviations/percentages/or deltas from where it must be
    public enum TrajType {
        kGetPiece,
        kShootPiece,
        kTrapMovement
    }
    public String name;
    public TrajType type;
    public ArrayList<Integer> keyPoints;
    public ArrayList<String> keyPointNames;

    public ArrayList<Boolean> atKeyPoint;

    public ArrayList<Pose2d> trajectory;
    public ArrayList<Twist2d> velocity;
    public ArrayList<Twist2d> acceleration;
    
    public double deltaTimePerPoint;

    public Integer currentPoint;

    public String targetID;

    public class SingleTrajectoryValue {
        public String name;
        public ArrayList<Integer> keyPoints;
        public ArrayList<String> keyPointNames;
        public ArrayList<Pose2d> trajectory;
        public ArrayList<Twist2d> acceleration;
        public ArrayList<Twist2d> velocity;
        public double deltaTimePerPoint;
    }

    public class CustomDeserializer extends StdDeserializer<SingleTrajectoryValue>  {

        protected CustomDeserializer(Class<?> vc) {
            super(vc);
            //TODO Auto-generated constructor stub
        }

        @Override
        public SingleTrajectoryValue deserialize(JsonParser p, DeserializationContext ctxt)
                throws IOException, JacksonException {
            // TODO Auto-generated method stub
            JsonNode node = p.getCodec().readTree(p);
            SingleTrajectoryValue traj = new SingleTrajectoryValue();
            traj.name = node.get("name").asText();
            traj.keyPoints = new ArrayList<Integer>();
            traj.keyPointNames = new ArrayList<String>();
            traj.trajectory = new ArrayList<Pose2d>();
            traj.acceleration = new ArrayList<Twist2d>();
            traj.velocity = new ArrayList<Twist2d>();
            traj.deltaTimePerPoint = node.get("deltaTimePerPoint").asDouble();
            for (JsonNode keyPoint : node.get("keyPoints")){
                traj.keyPoints.add(keyPoint.asInt());
            }
            for (JsonNode keyPointName : node.get("keyPointNames")){
                traj.keyPointNames.add(keyPointName.asText());
            }
            for (JsonNode pose : node.get("trajectory")){
                traj.trajectory.add(new Pose2d(pose.get("translation").get("x").asDouble(), pose.get("translation").get("y").asDouble(), new Rotation2d(pose.get("rotation").get("yaw").asDouble())));
            }
            for (JsonNode twist : node.get("velocity")){
                traj.velocity.add(new Twist2d(twist.get("dx").asDouble(), twist.get("dy").asDouble(), twist.get("dtheta").asDouble()));
            }
            for (JsonNode twist : node.get("acceleration")){
                traj.acceleration.add(new Twist2d(twist.get("dx").asDouble(), twist.get("dy").asDouble(), twist.get("dtheta").asDouble()));
            }
            return traj;
            
                    
        }

    }

    
    public TrajectoryManager(String name, TrajType type){
        this.name = name;
        this.type = type;
        keyPoints = new ArrayList<Integer>();
        keyPointNames = new ArrayList<String>();
        atKeyPoint = new ArrayList<Boolean>();
        trajectory = new ArrayList<Pose2d>();
        velocity = new ArrayList<Twist2d>();
        deltaTimePerPoint = 0.02;
        currentPoint = 0;
        if(!getTrajectoryFromNetworkTables()){
            
        }

    }

    public void setTrajectoryToPlanFor(TrajType type){
        // set in network tables the type of trajectory to plan for
        this.type = type;
    }


    private boolean getTrajectoryFromNetworkTables(){
        // get the trajectory from network tables
        // deserialize it and capture keypoints, names, and other relevant data        
        // get TrajectoryGenerationManager.kTrajectoryTableName from network tables
        
        NetworkTable table = NetworkTableInstance.getDefault().getTable(TrajectoryGenerationManager.kTrajectoryTableName);
        if(table.containsKey(name)){
            GenericSubscriber trajectoryJson = table.getStringTopic(name).genericSubscribe(PubSubOption.pollStorage(1));
            String json = trajectoryJson.getString(null);
            if (json != null){
                ObjectMapper trajectory = new ObjectMapper();
                try {
                    SingleTrajectoryValue traj = trajectory.readValue(json, SingleTrajectoryValue.class);
                    this.keyPoints = traj.keyPoints;
                    this.keyPointNames = traj.keyPointNames;
                    this.trajectory = traj.trajectory;
                    this.velocity = traj.velocity;
                    this.acceleration = traj.acceleration;
                    this.deltaTimePerPoint = traj.deltaTimePerPoint;
                    this.targetID = traj.name;
                    for (int i = 0; i < keyPoints.size(); i++){
                        atKeyPoint.add(false);
                    }
                    
                    Logger.recordOutput("TrajectoryOutputs/"+name, TrajectoryGenerator.generateTrajectory(traj.trajectory, new TrajectoryConfig(Constants.DriveConstants.kMaxSpeedMetersPerSecond, Constants.DriveConstants.kMaxAccelMetersPerSecondSq)));
                    return true;
                } catch (Exception e){
                    return false;
                }
            }
        }

        return false;
    }

    // return if the robot has passed a keypoint and if the keypoint is still false, set it to true
    public boolean atKeyPoint(int keyPoint){
        for (int i = 0; i < keyPoints.size(); i++){
            if (currentPoint>= keyPoints.get(i) ){
                if (atKeyPoint.get(i) == false){
                    atKeyPoint.set(i, true);
                    return true;
                }
            }
        }
        return false;
    }
    
}


