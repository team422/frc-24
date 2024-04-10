package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoCurrentState {
    
    public enum AutoState {
        LookingForPiece,
        DrivingToPiece,
        GoingToShootPiece,
        ShootingPiece,
        DrivingToEnd
    }

    public Command driveToPiece;

    public Command driveToShoot;

    public Pose2d lastPiecePose;

    public AutoState currentState;

    public AutoCurrentState() {
        currentState = AutoState.LookingForPiece;
    }

    public void setState(AutoState state) {
        currentState = state;
    }

    public AutoState getState() {
        return currentState;
    }

    public void setDriveToPieceCommand(Command command) {
        driveToPiece = command;
    }

    public void setDriveToShootCommand(Command command) {
        driveToShoot = command;
    }

    







}
