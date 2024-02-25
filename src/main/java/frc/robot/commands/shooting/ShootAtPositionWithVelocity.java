package frc.robot.commands.shooting;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;

public class ShootAtPositionWithVelocity extends Command {
    
    Rotation2d m_shooterAngle;
    Pose2d m_position;
    Twist2d m_velocity;

    public ShootAtPositionWithVelocity(Pose2d position, Twist2d velocity) {


    
    }

    public ShootAtPositionWithVelocity() {
        // get entire path 
        System.out.println(RobotState.getInstance().activePath.size());
      
    }

    public void initialize() {
        if (m_position == null){
            System.out.println(RobotState.getInstance().activePath.size());
            // find where the target position is in the path
            Pose2d tag = RobotState.getInstance().currentTarget;
            // for (int i = 0; i < RobotState.getInstance().activePath.size(); i++){
            //     if (RobotState.getInstance().activePath.get(i)){
            //         m_position = RobotState.getInstance().activePath.get(i);
            //         System.out.println("Found target position in path");
            //     }
            // }
        }
    }

    public void execute() {
        System.out.println(RobotState.getInstance().activePath.size());
    }

}
