package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotState.RobotCurrentAction;
import frc.robot.subsystems.indexer.Indexer.IndexerState;

public class AutoShoot extends Command {
    double simTime = -1 ;
    public AutoShoot() {
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID constants here
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (Robot.isSimulation()){
            simTime = Timer.getFPGATimestamp();
        }
        frc.robot.RobotState.getInstance().setRobotCurrentAction(RobotCurrentAction.kAutoShoot);
    }
    @Override
    public boolean isFinished() {
        if (Robot.isSimulation()){
            if(Timer.getFPGATimestamp() - simTime > 1){
                frc.robot.RobotState.getInstance().setRobotCurrentAction(RobotCurrentAction.kPathPlanner);
                return true;
            }
        }
        frc.robot.RobotState.getInstance().setIndexer(IndexerState.INDEXING);
        return frc.robot.RobotState.getInstance().curAction != RobotCurrentAction.kAutoShoot;
    }

    


    
}
