package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotCurrentAction;
import frc.robot.subsystems.drive.Drive.DriveProfiles;

public class AutoIntakeDeadline extends Command {
    public double timeLimit = 0.0;
    private boolean left;

    public AutoIntakeDeadline(boolean left) {
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID constants here
        this.left = left;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
            RobotState.getInstance().autoIntakeLeftOrRight = left ? -1 : 1;
        } else {
            RobotState.getInstance().autoIntakeLeftOrRight = left ? 1 : -1;
        }
        RobotState.getInstance().setRobotCurrentAction(RobotCurrentAction.kAutoAutoIntake);
        timeLimit = Timer.getFPGATimestamp() + 1;
    }

    @Override
    public boolean isFinished() {
        return RobotState.getInstance().curAction != RobotCurrentAction.kAutoAutoIntake;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            RobotState.getInstance().setRobotCurrentAction(RobotCurrentAction.kPathPlanner);
            RobotState.getInstance().getDrive().setProfile(DriveProfiles.kTrajectoryFollowing);
        }
    }
}
