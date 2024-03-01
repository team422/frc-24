package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState.RobotCurrentAction;

public class AutoIntake extends Command {
        
        public AutoIntake() {
            // Use addRequirements() here to declare subsystem dependencies.
            // Configure additional PID constants here
        }
    
        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
            frc.robot.RobotState.getInstance().setRobotCurrentAction(RobotCurrentAction.kAutoIntake);
        }
        @Override
        public boolean isFinished() {
            return frc.robot.RobotState.getInstance().curAction != RobotCurrentAction.kAutoIntake;
        }
}
