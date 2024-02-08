package frc.robot.utils;

import java.util.ArrayList;

import org.apache.commons.math3.geometry.euclidean.threed.Plane;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.FieldConstants;

public class ShooterMath {

    double targetBaseHeight = FieldConstants.kShooterBaseHeight;
    double targetTopHeight = FieldConstants.kShooterTopHeight;

    double targetDepth = FieldConstants.kShooterDepth;

    Rotation2d targetAngle = FieldConstants.kShooterAngle;

    



    double shooterHeight; // this is the height of the pivot
    double shooterDistance; // this is the distance from the pivot to the end of the shooter


    public ShooterMath(double shooterHeight) {
        this.shooterHeight = shooterHeight;
    }

    // for right now, we are going to assume the note moves linearly
    // we will need to change this later
    public ArrayList<Rotation2d> getShooterAngle (Pose3d targetPose, Pose3d shooterPose) {
        ArrayList<Rotation2d> angles = new ArrayList<Rotation2d>();
        // First is yaw angle of the robot to the target
        // second is the angle of the elevation angle to the target
        
        // we need to find the angle of the target relative to the shooter

        double deltaHeightMetersoverDepthMeters = (targetPose.getZ()-shooterPose.getZ())/targetPose.minus(shooterPose).getTranslation().getNorm();

        Rotation2d elevation = Rotation2d.fromRadians(Math.asin(deltaHeightMetersoverDepthMeters));
        Rotation2d yaw = Rotation2d.fromRadians(Math.atan2(targetPose.getTranslation().minus(shooterPose.getTranslation()).getY(), targetPose.getTranslation().minus(shooterPose.getTranslation()).getX())); 
        

        Logger.recordOutput("Shooter Math", new Pose2d(shooterPose.toPose2d().getTranslation(), yaw));

        Transform3d shootingTranslation = targetPose.minus(shooterPose);
        // System.out.println(shootingTranslation);
        // System.out.println(yaw.getDegrees());
        // System.out.println(elevation.getDegrees());
        angles.add(yaw);
        angles.add(elevation);
        return angles;


    }
}
