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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotState;
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

    InterpolatingDoubleTreeMap m_shootHockeyPuckLeft = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap m_shootHockeyPuckRight = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap m_hockeyPuckDistanceAngle = new InterpolatingDoubleTreeMap();




    Translation3d shooterTranslation; // this is the height of the pivot
    double shooterDistance; // this is the distance from the pivot to the end of the shooter


    public ShooterMath(Translation3d shooterTranslation) { // translation should be to the pivot of the shooter
        this.shooterTranslation = shooterTranslation;
        setUpTreeMap();
    }

    public void setUpTreeMap(){
        // 37.625
        double constantAddition = Units.inchesToMeters(50.5);
        m_distanceAngle.put(Units.inchesToMeters(0)+constantAddition,55.0);
        m_distanceAngle.put(Units.inchesToMeters(12)+constantAddition,49.0);
        m_distanceAngle.put(Units.inchesToMeters(24)+constantAddition,47.0);
        m_distanceAngle.put(Units.inchesToMeters(36)+constantAddition,43.0);
        m_distanceAngle.put(Units.inchesToMeters(48)+constantAddition,38.5);
        m_distanceAngle.put(Units.inchesToMeters(60)+constantAddition,35.0);
        m_distanceAngle.put(Units.inchesToMeters(72)+constantAddition,34.0);
        m_distanceAngle.put(Units.inchesToMeters(84)+constantAddition,31.5);
        m_distanceAngle.put(Units.inchesToMeters(96)+constantAddition,30.00000);
        m_distanceAngle.put(Units.inchesToMeters(108)+constantAddition,29.000000);
        m_distanceAngle.put(Units.inchesToMeters(120)+constantAddition,28.0);
        m_distanceAngle.put(Units.inchesToMeters(140)+constantAddition,24.5);
        m_distanceAngle.put(Units.inchesToMeters(160)+constantAddition,23.);
        m_distanceAngle.put(Units.inchesToMeters(180)+constantAddition,21.75);
        m_distanceAngle.put(Units.inchesToMeters(200)+constantAddition,21.25);
        m_distanceAngle.put(Units.inchesToMeters(220)+constantAddition,20.8);


        m_hockeyPuckDistanceAngle.put(10.47,37.0);
        m_hockeyPuckDistanceAngle.put(9.9,41.0);
        m_hockeyPuckDistanceAngle.put(9.57,45.0);
        m_hockeyPuckDistanceAngle.put(9.07,43.0);
        m_hockeyPuckDistanceAngle.put(7.9,52.0);
        m_hockeyPuckDistanceAngle.put(7.22,65.0);
        m_hockeyPuckDistanceAngle.put(5.5,59.0);
        // m_hockeyPuckDistanceAngle.put(Units.inchesToMeters(300),50.0);
        m_shootHockeyPuckLeft.put(10.47,8.0);
        m_shootHockeyPuckLeft.put(9.9,7.5);
        m_shootHockeyPuckLeft.put(9.57,7.0);
        m_shootHockeyPuckLeft.put(9.07,7.0);
        m_shootHockeyPuckLeft.put(7.9,7.0);
        m_shootHockeyPuckLeft.put(7.22,9.0);
        m_shootHockeyPuckLeft.put(5.5,9.0);
        m_shootHockeyPuckRight.put(10.47,13.0);
        m_shootHockeyPuckRight.put(9.9,12.0);
        m_shootHockeyPuckRight.put(9.57,11.5);
        m_shootHockeyPuckRight.put(9.07,11.0);
        m_shootHockeyPuckRight.put(7.9,11.0);
        m_shootHockeyPuckRight.put(7.22,12.0);
        m_shootHockeyPuckRight.put(5.55,11.0);
        
        
        
        
        
        
        m_shootSpeedRight.put(Units.inchesToMeters(0)+constantAddition,13.0);
        m_shootSpeedRight.put(Units.inchesToMeters(12)+constantAddition,13.0);
        m_shootSpeedRight.put(Units.inchesToMeters(24)+constantAddition,13.0);
        m_shootSpeedRight.put(Units.inchesToMeters(36)+constantAddition,13.0);
        m_shootSpeedRight.put(Units.inchesToMeters(48)+constantAddition,13.0);
        m_shootSpeedRight.put(Units.inchesToMeters(60)+constantAddition,13.0);
        m_shootSpeedRight.put(Units.inchesToMeters(72)+constantAddition,13.);
        m_shootSpeedRight.put(Units.inchesToMeters(84)+constantAddition,13.0);
        m_shootSpeedRight.put(Units.inchesToMeters(96)+constantAddition,14.0);
        m_shootSpeedRight.put(Units.inchesToMeters(108)+constantAddition,14.0);
        m_shootSpeedRight.put(Units.inchesToMeters(120)+constantAddition,14.0);
        m_shootSpeedRight.put(Units.inchesToMeters(140)+constantAddition,14.0);
        m_shootSpeedRight.put(Units.inchesToMeters(160)+constantAddition,14.5);
        m_shootSpeedRight.put(Units.inchesToMeters(180)+constantAddition,15.0);
        m_shootSpeedRight.put(Units.inchesToMeters(200)+constantAddition,15.0);
        m_shootSpeedRight.put(Units.inchesToMeters(220)+constantAddition,16.0);
        // m_shootSpeedRight.put(Units.inchesToMeters(100.625)+constantAddition,15.0);
        // m_shootSpeedRight.put(Units.inchesToMeters(116.625)+constantAddition,15.0);
        // m_shootSpeedRight.put(Units.inchesToMeters(130.625)+constantAddition,15.0);
        // m_shootSpeedRight.put(Units.inchesToMeters(143.625)+constantAddition,17.0);
        // m_shootSpeedRight.put(Units.inchesToMeters(160.625)+constantAddition,17.0);
        // m_shootSpeedRight.put(Units.inchesToMeters(175.625)+constantAddition,21.0);
        // m_shootSpeedRight.put(Units.inchesToMeters(200.625)+constantAddition,18.000000);
        
        m_shootSpeedLeft.put(Units.inchesToMeters(0)+constantAddition,10.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(12)+constantAddition,10.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(24)+constantAddition,10.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(36)+constantAddition,10.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(48)+constantAddition,10.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(60)+constantAddition,10.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(72)+constantAddition,10.);
        m_shootSpeedLeft.put(Units.inchesToMeters(84)+constantAddition,10.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(96)+constantAddition,11.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(108)+constantAddition,11.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(120)+constantAddition,11.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(140)+constantAddition,11.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(160)+constantAddition,11.5);
        m_shootSpeedLeft.put(Units.inchesToMeters(180)+constantAddition,12.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(200)+constantAddition,12.0);
        m_shootSpeedLeft.put(Units.inchesToMeters(220)+constantAddition,12.5);
        // m_shootSpeedLeft.put(Units.inchesToMeters(116.625)+constantAddition,13.0);
        // m_shootSpeedLeft.put(Units.inchesToMeters(130.625)+constantAddition,13.0);
        // m_shootSpeedLeft.put(Units.inchesToMeters(143.625)+constantAddition,15.000000);
        // m_shootSpeedLeft.put(Units.inchesToMeters(160.625)+constantAddition,15.0);
        // m_shootSpeedLeft.put(Units.inchesToMeters(175.625)+constantAddition,19.0);
        // m_shootSpeedLeft.put(Units.inchesToMeters(200.625)+constantAddition,16.000000);

        


    }


    public ArrayList<Rotation2d> setNextShootingPoseAndVelocityFeeder(Pose2d plannedRobotPose, Twist2d plannedSpeed ,Translation3d target3d){
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
        // Logger.recordOutput("Rise",rise);
        Logger.recordOutput("run",run);
        Rotation2d yaw = Rotation2d.fromRadians(Math.atan2(newTarget.getY()-shooterPosition.getTranslation().getY(), newTarget.getX()-shooterPosition.getTranslation().getX()));
        // Rotation2d elevation = Rotation2d.fromRadians(Math.asin((newTarget.getZ()-shooterPosition.getZ())/newTarget.minus(shooterPosition.getTranslation()).getNorm()));
        // Rotation2d elevation = Rotation2d.fromRadians(Math.atan(rise/run));
        // Rotation2d elevation = Rotation2d.fromDegrees(m_distanceAngle.get(run));
        Rotation2d elevation = Rotation2d.fromDegrees(m_hockeyPuckDistanceAngle.get(run));
        // Double speedLeft = 
        // Double speedRight = 
        // Logger.recordOutput("Shooter Math", new Pose2d(shooterPosition.toPose2d().getTranslation(), yaw));
        // Logger.recordOutput("Shooter Elevation", elevation);
        // Logger.recordOutput("ShooterMath", newRobotPose);
        ArrayList<Rotation2d> vals = new ArrayList<>();
        vals.add(yaw);
        vals.add(elevation);
        return vals;



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
        // Logger.recordOutput("Rise",rise);
        Logger.recordOutput("run",run);
        Rotation2d yaw = Rotation2d.fromRadians(Math.atan2(newTarget.getY()-shooterPosition.getTranslation().getY(), newTarget.getX()-shooterPosition.getTranslation().getX()));
        // Rotation2d elevation = Rotation2d.fromRadians(Math.asin((newTarget.getZ()-shooterPosition.getZ())/newTarget.minus(shooterPosition.getTranslation()).getNorm()));
        // Rotation2d elevation = Rotation2d.fromRadians(Math.atan(rise/run));
        // Rotation2d elevation = Rotation2d.fromDegrees(m_distanceAngle.get(run));
        
        Rotation2d elevation = Rotation2d.fromDegrees(m_distanceAngle.get(run));
        // Rotation2d elevation = Rotation2d.fromDegrees(calculateShooterElevation(run));
        // Double speedLeft = 
        // Double speedRight = 
        // Logger.recordOutput("Shooter Math", new Pose2d(shooterPosition.toPose2d().getTranslation(), yaw));
        // Logger.recordOutput("Shooter Elevation", elevation);
        // Logger.recordOutput("ShooterMath", newRobotPose);
        ArrayList<Rotation2d> vals = new ArrayList<>();
        vals.add(yaw);
        vals.add(elevation);
        return vals;



    }

    // public double calculateShooterElevation(double distance){
    //     distance = Units.metersToInches(distance);
    //     double A = 0.00000234516;
    //     double B = -0.000459409;
    //     double C = -0.21339;
    //     double D = 66.5361;
    //     if (distance > 252) return 21.0;
    //     return distance * (distance * (distance * A + B) + C) + D;
    // }


    public ArrayList<Double> getShooterMetersPerSecondFeeder(double distance){
        ArrayList<Double> speeds = new ArrayList<>();
        speeds.add(m_shootHockeyPuckLeft.get(distance));
        speeds.add(m_shootHockeyPuckRight.get(distance));
        return speeds;
    }

    public double calculateShootingHeadingTolerance(double distance){
        distance = Units.metersToInches(distance);
        return Math.max(ShooterMathConstants.headingErrorMultiplier.get() *(ShooterMathConstants.errorCubicA*Math.pow(distance, 3) +ShooterMathConstants.errorCubicB*Math.pow(distance, 2) + ShooterMathConstants.errorCubicC*Math.pow(distance, 1) + ShooterMathConstants.errorCubicD ),1.0); 
    }

    public double calculateShootingPivotTolerance( double distance ){
        // distance = Units.metersToInches(distance);
        // return (ShooterMathConstants.errorCubicA*Math.pow(distance, 3) +ShooterMathConstants.errorCubicB*Math.pow(distance, 2) + ShooterMathConstants.errorCubicC*Math.pow(distance, 1) + ShooterMathConstants.errorCubicD)/ShooterMathConstants.pivotErrorMultiplier.get() ;
        distance = Units.metersToInches(distance);
        return Math.max(ShooterMathConstants.PivotDropoffMultiplier.get()*distance+ShooterMathConstants.PivotDropoffInitilizer.get(),ShooterMathConstants.PivotDropoffFloor.get());
    }

    public double calculateAcceptableDropoff(double distance){
        distance = Units.metersToInches(distance);
        return Math.max(ShooterMathConstants.FlywheelDropoffMultiplier.get()*distance+ShooterMathConstants.FlywheelDropoffInitilizer.get(),ShooterMathConstants.FlywheelDropoffFloor.get());
    }




    public ArrayList<Double> getShooterMetersPerSecond(double distance){
        ArrayList<Double> vals = new ArrayList<>();
        DriverStation.getAlliance().ifPresentOrElse((Alliance alliance)->{

        if(alliance.equals(Alliance.Blue)){
            vals.add(m_shootSpeedLeft.get(distance));
            vals.add(m_shootSpeedRight.get(distance));
        }else{
            vals.add(m_shootSpeedRight.get(distance));
            vals.add(m_shootSpeedLeft.get(distance));
        }
        }, ()->{
            vals.add(m_shootSpeedLeft.get(distance));
            vals.add(m_shootSpeedRight.get(distance));
        });
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
