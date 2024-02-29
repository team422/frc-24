package frc.robot.utils;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterMathConstants;

public class ShooterMath {

    double targetBaseHeight = FieldConstants.kShooterBaseHeight;
    double targetTopHeight = FieldConstants.kShooterTopHeight;

    double targetDepth = FieldConstants.kShooterDepth;

    Rotation2d targetAngle = FieldConstants.kShooterAngle;

    // 3d position middle left of the speaker from field constants
    Translation3d middleOfSpeaker =  FieldConstants.kShooterCenter;

    InterpolatingDoubleTreeMap m_distanceAngle = new InterpolatingDoubleTreeMap();

    InterpolatingDoubleTreeMap m_shootSpeedRight = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap m_shootSpeedLeft = new InterpolatingDoubleTreeMap();




    Translation3d shooterTranslation; // this is the height of the pivot
    double shooterDistance; // this is the distance from the pivot to the end of the shooter


    public ShooterMath(Translation3d shooterTranslation) { // translation should be to the pivot of the shooter
        this.shooterTranslation = shooterTranslation;
        setUpTreeMap();
    }

    public void setUpTreeMap(){
        // 37.625
        double constantAddition = Units.inchesToMeters(36+15.5);
        m_distanceAngle.put(0.0+constantAddition,61.0);
        m_distanceAngle.put(Units.inchesToMeters(12.0)+constantAddition,51.0);
        m_distanceAngle.put(Units.inchesToMeters(36.125)+constantAddition,45.0);
        m_distanceAngle.put(Units.inchesToMeters(47.125)+constantAddition,42.0);
        m_distanceAngle.put(Units.inchesToMeters(63.25)+constantAddition,39.0);
        m_distanceAngle.put(Units.inchesToMeters(79.5)+constantAddition,37.0);
        m_distanceAngle.put(Units.inchesToMeters(94.5)+constantAddition,34.0);
        m_distanceAngle.put(Units.inchesToMeters(113.625)+constantAddition,26.0);
        m_distanceAngle.put(Units.inchesToMeters(133.625)+constantAddition,24.0);
        m_distanceAngle.put(Units.inchesToMeters(155.625)+constantAddition,23.0);
        m_distanceAngle.put(Units.inchesToMeters(173.625)+constantAddition,22.5);
        // m_distanceAngle.put(Units.inchesToMeters(173.625)+constantAddition,23.0);
        
        
        
        
        
        
        m_shootSpeedRight.put(0.0+constantAddition,15.0);
        m_shootSpeedRight.put(Units.inchesToMeters(12.0)+constantAddition,18.0);
        m_shootSpeedRight.put(Units.inchesToMeters(23.75)+constantAddition,18.0);
        m_shootSpeedRight.put(Units.inchesToMeters(36.125)+constantAddition,20.0);
        m_shootSpeedRight.put(Units.inchesToMeters(47.125)+constantAddition,20.0);
        m_shootSpeedRight.put(Units.inchesToMeters(63.25)+constantAddition,18.0);
        m_shootSpeedRight.put(Units.inchesToMeters(79.5)+constantAddition,18.0);
        m_shootSpeedRight.put(Units.inchesToMeters(94.5)+constantAddition,19.0);
        m_shootSpeedRight.put(Units.inchesToMeters(113.625)+constantAddition,28.0);
        m_shootSpeedRight.put(Units.inchesToMeters(133.625)+constantAddition,28.0);
        m_shootSpeedRight.put(Units.inchesToMeters(155.625)+constantAddition,28.0);
        // m_shootSpeedRight.put(Units.inchesToMeters(173.625)+constantAddition,28.0);
        // m_shootSpeedRight.put(Units.inchesToMeters(173.625)+constantAddition,32.0);
        
        m_shootSpeedLeft.put(0.0+constantAddition,15.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(12.0)+constantAddition,18.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(23.75)+constantAddition,18.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(36.125)+constantAddition,16.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(47.125)+constantAddition,16.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(63.25)+constantAddition,14.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(79.5)+constantAddition,14.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(94.25)+constantAddition,15.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(113.625)+constantAddition,24.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(133.625)+constantAddition,24.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(155.625)+constantAddition,24.0);
        // m_shootSpeedLeft.put(Units.inchesToMeters(173.625)+constantAddition,24.0);
        // m_shootSpeedLeft.put(Units.inchesToMeters(173.625)+constantAddition,31.0);
        


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
        double speed = (getShooterMetersPerSecond(distance).get(0) + getShooterMetersPerSecond(distance).get(1))/2.0;

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
        // Rotation2d elevation = Rotation2d.fromRadians(Math.atan(rise/run));
        // Rotation2d elevation = Rotation2d.fromDegrees(m_distanceAngle.get(run));
        Rotation2d elevation = Rotation2d.fromDegrees(m_distanceAngle.get(run));
        // Double speedLeft = 
        // Double speedRight = 
        Logger.recordOutput("Shooter Math", new Pose2d(shooterPosition.toPose2d().getTranslation(), yaw));
        Logger.recordOutput("Shooter Elevation", elevation);
        // Logger.recordOutput("ShooterMath", newRobotPose);
        ArrayList<Rotation2d> vals = new ArrayList<>();
        vals.add(yaw);
        vals.add(elevation);
        return vals;



    }



    public ArrayList<Double> getShooterMetersPerSecond(double distance){
        ArrayList<Double> vals = new ArrayList<>();
        vals.add(m_shootSpeedLeft.get(distance));
        vals.add(m_shootSpeedRight.get(distance));
        return vals;
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
