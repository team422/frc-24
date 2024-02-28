package frc.robot.utils;

import java.util.ArrayList;

import org.apache.commons.math3.geometry.euclidean.threed.Plane;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterMathConstants;

public class ShooterMath {

    double targetBaseHeight = FieldConstants.kShooterBaseHeight;
    double targetTopHeight = FieldConstants.kShooterTopHeight;

    double targetDepth = FieldConstants.kShooterDepth;

    Rotation2d targetAngle = FieldConstants.kShooterAngle;

    // 3d position middle left of the speaker from field constants
    Translation3d middleOfSpeaker =  FieldConstants.kShooterCenter;





    Translation3d shooterTranslation; // this is the height of the pivot
    double shooterDistance; // this is the distance from the pivot to the end of the shooter


    public ShooterMath(Translation3d shooterTranslation) { // translation should be to the pivot of the shooter
        this.shooterTranslation = shooterTranslation;
    }

    public ArrayList<Rotation2d> setNextShootingPoseAndVelocity(Pose2d plannedRobotPose, Twist2d plannedSpeed ,Translation3d target3d){
        // assume the rotation of the planned robot pose is incorrect
        // first find the angle the robot would have to be at to face the target
        Rotation2d robotYaw = Rotation2d.fromRadians(Math.atan2(target3d.getY()-plannedRobotPose.getTranslation().getY(), target3d.getX()-plannedRobotPose.getTranslation().getX()));
        Pose2d newRobotPose = new Pose2d(plannedRobotPose.getTranslation(), robotYaw);
        // now get the position of the shooter by adding the shooter translation to the robot pose3d
        Pose3d shooterPosition = new Pose3d(newRobotPose).plus(new Transform3d(shooterTranslation, new Rotation3d()));
        // now we get the distance from the shooter to the target

        double distance = shooterPosition.getTranslation().getDistance(target3d);
        double speed = getShooterMetersPerSecond(distance);

        double timeOfFlight = distance/speed;
        // System.out.println(speed);

        // now multiply this by the twist2d to get how much the velocity of the robot should impact the final position of the note
        Translation3d deltaPosition = new Translation3d(plannedSpeed.dx*timeOfFlight, plannedSpeed.dy*timeOfFlight, 0);

        // now add this to the target position
        Translation3d newTarget = target3d.minus(deltaPosition);

        // now we need to find the angle of the target relative to the shooter
        double rise = target3d.getZ() - shooterPosition.getZ();
        double run = new Pose2d(target3d.getX(),target3d.getY(),new Rotation2d()).minus(newRobotPose).getTranslation().getNorm();
        Logger.recordOutput("Rise",rise);
        Logger.recordOutput("run",run);
        Rotation2d yaw = Rotation2d.fromRadians(Math.atan2(newTarget.getY()-shooterPosition.getTranslation().getY(), newTarget.getX()-shooterPosition.getTranslation().getX()));
        // Rotation2d elevation = Rotation2d.fromRadians(Math.asin((newTarget.getZ()-shooterPosition.getZ())/newTarget.minus(shooterPosition.getTranslation()).getNorm()));
        Rotation2d elevation = Rotation2d.fromRadians(Math.atan(rise/run));
        Logger.recordOutput("Shooter Math", new Pose2d(shooterPosition.toPose2d().getTranslation(), yaw));
        // Logger.recordOutput("ShooterMath", newRobotPose);
        ArrayList<Rotation2d> vals = new ArrayList<>();
        vals.add(yaw);
        vals.add(elevation);
        return vals;



    }

    public double getShooterMetersPerSecond(double distance){
        return ShooterMathConstants.cubicA * Math.pow(distance, 3) + ShooterMathConstants.cubicB * Math.pow(distance, 2) + ShooterMathConstants.cubicC * distance + ShooterMathConstants.cubicD;
    }


    public double getDistanceFromTarget(Pose2d pose,Translation3d target){
        return new Pose2d(target.getX(),target.getY(),new Rotation2d()).minus(pose).getTranslation().getNorm();

    }



    // // for right now, we are going to assume the note moves linearly
    // // we will need to change this later
    // public ArrayList<Rotation2d> getShooterAngle (Pose3d targetPose, Pose3d shooterPose) {
    //     ArrayList<Rotation2d> angles = new ArrayList<Rotation2d>();
    //     // First is yaw angle of the robot to the target
    //     // second is the angle of the elevation angle to the target
        
    //     // we need to find the angle of the target relative to the shooter

    //     double deltaHeightMetersoverDepthMeters = (targetPose.getZ()-shooterPose.getZ())/targetPose.minus(shooterPose).getTranslation().getNorm();

    //     Rotation2d elevation = Rotation2d.fromRadians(Math.asin(deltaHeightMetersoverDepthMeters));
    //     Rotation2d yaw = Rotation2d.fromRadians(Math.atan2(targetPose.getTranslation().minus(shooterPose.getTranslation()).getY(), targetPose.getTranslation().minus(shooterPose.getTranslation()).getX())); 
        

    //     Logger.recordOutput("Shooter Math", new Pose2d(shooterPose.toPose2d().getTranslation(), yaw));

    //     Transform3d shootingTranslation = targetPose.minus(shooterPose);
    //     // System.out.println(shootingTranslation);
    //     // System.out.println(yaw.getDegrees());
    //     // System.out.println(elevation.getDegrees());
    //     angles.add(yaw);
    //     angles.add(elevation);
    //     return angles;
    // }
}
