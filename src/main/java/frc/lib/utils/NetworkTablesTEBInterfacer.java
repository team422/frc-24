package frc.lib.utils;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import frc.robot.RobotState;

public class NetworkTablesTEBInterfacer {
    final String kTopicName = "/trajectory_generator/output";
    DoubleArraySubscriber m_subscriberTrajectory;
    DoubleArrayPublisher m_publishPose;
    DoubleArrayPublisher m_publishDesiredPose;
    CustomTraj m_deserializedTrajectory;

    public NetworkTablesTEBInterfacer(String identifier) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(kTopicName);
        var configTable = table.getSubTable("config");
        // configTable.getDoubleArrayTopic("pose").publish().set(convertPoseToDoubleArray(RobotState.getInstance().getRobotPose()));
        m_publishPose = table.getDoubleArrayTopic("pose").publish(PubSubOption.keepDuplicates(true));
        m_publishDesiredPose = table.getDoubleArrayTopic("desired_pose").publish(PubSubOption.keepDuplicates(true));
        m_subscriberTrajectory = table.getDoubleArrayTopic(kTopicName).subscribe(new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));

    }

    public double[] convertPoseToDoubleArray(Pose2d pose) {
        double[] res = new double[3];
        res[0] = pose.getTranslation().getX();
        res[1] = pose.getTranslation().getY();
        res[2] = pose.getRotation().getRadians();
        return res;
    }


    public void update(){
        m_publishPose.set(convertPoseToDoubleArray(RobotState.getInstance().getRobotPose()));
        m_publishDesiredPose.set(convertPoseToDoubleArray(RobotState.getInstance().getDesiredRobotPose()));
    }

    public ArrayList<TrajectoryPoint> deserialize() {
        double[] curr = m_subscriberTrajectory.get();
        
        ArrayList<TrajectoryPoint> res = new ArrayList<TrajectoryPoint>();
        
        for (int i = 0; i < curr.length; i += 9) {
            res.add(new TrajectoryPoint(new Pose2d(curr[i], curr[i + 1], new Rotation2d(curr[i + 2])), new Twist2d(curr[i + 3], curr[i + 4], curr[i + 5]), new Twist2d(curr[i + 6], curr[i + 7], curr[i + 8])));
        }

        return res;
    }
}