package frc.lib.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;

public class TrajectoryPoint {
    public Pose2d pose;
    public Twist2d velocity;
    public Twist2d acceleration;
    public Double time;

    public TrajectoryPoint(Pose2d pose, Twist2d velocity, Twist2d acceleration, Double time) {
        this.pose = pose;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.time = time;
    }

    public TrajectoryPoint(Pose2d pose, Twist2d velocity) {
        this.pose = pose;
        this.velocity = velocity;
        this.acceleration = new Twist2d();
        this.time = 0.0;
    }

    public TrajectoryPoint(Pose2d pose) {
        this.pose = pose;
        this.velocity = new Twist2d();
        this.acceleration = new Twist2d();
        this.time = 0.0;
    }

    public TrajectoryPoint(Pose2d pose, Twist2d velocity, Twist2d acceleration) {
        this.pose = pose;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.time = 0.0;
    }

}
