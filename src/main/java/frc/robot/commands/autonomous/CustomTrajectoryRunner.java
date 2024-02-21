package frc.robot.commands.autonomous;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utils.TrajectoryPoint;
import frc.robot.FieldConstants;

public class CustomTrajectoryRunner extends Command {
    String[] m_positionsToGoTo;
    int currentPathPointIndex = 0;
    ArrayList<String> m_pathNamesReached;
    String m_currentPathName;
    public CustomTrajectoryRunner() {
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling getController() and getRamseteCommand() and setting the desired options
        m_positionsToGoTo = new String[]{"CloseRight"};
        m_pathNamesReached = new ArrayList<String>();
        m_currentPathName = m_positionsToGoTo[0];

    }

    public TrajectoryPoint getNextPose(){
        // return the next pose in the path
        Pose2d pose = FieldConstants.getGamePieces().get(m_positionsToGoTo[0]);
        // Twist2d velocity = new Twist2d(m_basePath.getAllPathPoints().get(currentPathPointIndex)., m_basePath.g   etAllPathPoints().get(currentPathPointIndex).rotationTarget.getTarget());
        return new TrajectoryPoint(pose);
        
    }

    public void update(){
        // are we at desired pose?
        // if so, move to next pose
        // if not, do nothing
        Pose2d currentPose = new Pose2d();
        if(currentPose.getTranslation().getDistance(getNextPose().pose.getTranslation()) < 0.1 && Math.abs(currentPose.getRotation().getDegrees()-getNextPose().pose.getRotation().getDegrees())< 5){
            currentPathPointIndex++;
        }

    }


    
}
