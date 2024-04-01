package frc.robot.utils;

import java.util.ArrayList;
import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState.RobotCurrentAction;
import frc.robot.subsystems.drive.Drive.DriveProfiles;
import frc.robot.Robot;
import frc.robot.RobotState;



public class AutoBuilderManager {
    // number of notes to scoute
    public int num_scouts;

    // order of notes to go to

    public Note[] notes;

    // notes at the end

    public Note[] endNotes;
    public int currentNote = -1;
    public int notesShot = 0;

    public Command mDriveToPiece;
    public Command mScoutingCommand;
    

    public AutoState currAutoState = AutoState.STARTING;

    public Pose2d endingPose = new Pose2d(7.76,1.13,new Rotation2d());

    

    public Pose2d curNotePose;

    public AutoBuilderManager( ) {
        
    }

    public void setNumScouts(int num) {
        num_scouts = num;
    }

    public void setNotes(Note[] notes) {
        this.notes = notes;
    }

    public void setEndNotes(Note[] notes) {
        this.endNotes = notes;
    }

    public void update(){
        Logger.recordOutput("AutoBuilderState",currAutoState);
        switch (currAutoState) {
            case STARTING:
                mDriveToPiece = RobotState.getInstance().getAutoFactory().generateTrajectoryToPose(AllianceFlipUtil.apply(getNextNotePosition(false)), DriveConstants.kDriveToPieceSpeed, false, RobotState.getInstance());
                mDriveToPiece.schedule();
                break;
                

            case SCANNING:
                RobotState.getInstance().getDrive().setProfile(DriveProfiles.kAutoPiecePickup);
                
                RobotState.getInstance().setRobotCurrentAction(RobotState.RobotCurrentAction.kIntake);
                if (isCameraWorking()) {
                    if (AutoScanLoop.isAtScanThreshold(notes[currentNote], RobotState.getInstance().getEstimatedPose())) {
                        if (isNoteDetected()) {
                            currAutoState = AutoState.DRIVING_TO_NOTE;
                            RobotState.getInstance().getDrive().setProfile(DriveProfiles.kAutoPiecePickup);

                            curNotePose = getVisibleNoteLocation();
                            if(mDriveToPiece !=null){
                                mDriveToPiece.cancel();
                            }
                            mDriveToPiece = RobotState.getInstance().getAutoFactory().generateTrajectoryToPose(AllianceFlipUtil.apply(curNotePose),DriveConstants.kAutoAlignToAmpSpeed,false,RobotState.getInstance());
                            mDriveToPiece.schedule();
                        } else {
                            if(mDriveToPiece !=null){
                                mDriveToPiece.cancel();
                            }
                            mDriveToPiece = RobotState.getInstance().getAutoFactory().generateTrajectoryToPose(AllianceFlipUtil.apply(getNextNotePosition(false)), DriveConstants.kAutoAlignToAmpSpeed, false, RobotState.getInstance());
                            mDriveToPiece.schedule();
                        }
                    }
                } else {
                    currAutoState = AutoState.DRIVING_TO_NOTE;
                    
                }
                break;
            case DRIVING_TO_NOTE:
                if(isCameraWorking()){
                    if (isNoteDetected()) {
                        Pose2d closest = getVisibleNoteLocation();
                        if(closest.getTranslation().getDistance(curNotePose.getTranslation()) > 0.7) {
                            if(mDriveToPiece !=null){
                                mDriveToPiece.cancel();
                            }
                            mDriveToPiece = RobotState.getInstance().getAutoFactory().generateTrajectoryToPose(AllianceFlipUtil.apply(closest), DriveConstants.kAutoAlignToAmpSpeed, false, RobotState.getInstance());
                            mDriveToPiece.schedule();
                        }

                        RobotState.getInstance().getDrive().setDriveTurnOverride(RobotState.getInstance().getShooterMath().setNextShootingPoseAndVelocity(RobotState.getInstance().getEstimatedPose(),new Twist2d(),new Translation3d(closest.getX(),closest.getY(),0)).get(0));
                        // Logic to check if new note pose is significantly different from old note pose here
                        // If so, generate new path
                    }
                }
                // Check note position and maybe regenerate path
                

                if (isHoldingNote()) {
                    currAutoState = AutoState.SHOOTING;
                    if(mDriveToPiece !=null){
                        mDriveToPiece.cancel();
                    }
                    mDriveToPiece = RobotState.getInstance().getAutoFactory().generateTrajectoryToPose(AllianceFlipUtil.apply(AutoScanLoop.shoot(notes[currentNote])), DriveConstants.kAutoAlignToAmpSpeed, false, RobotState.getInstance());
                    mDriveToPiece.schedule();
                }else if(curNotePose != null){
                 if (AllianceFlipUtil.apply(curNotePose).getTranslation().getDistance(RobotState.getInstance().getEstimatedPose().getTranslation()) < 0.1){
                    currAutoState = AutoState.SHOOTING;
                    if(mDriveToPiece !=null){
                        mDriveToPiece.cancel();
                    }
                    mDriveToPiece = RobotState.getInstance().getAutoFactory().generateTrajectoryToPose(AllianceFlipUtil.apply(AutoScanLoop.shoot(notes[currentNote])), DriveConstants.kAutoAlignToAmpSpeed, false, RobotState.getInstance());
                    mDriveToPiece.schedule();
                 }
                }
                
                break;
            case SHOOTING:
                // Start Shooter/ Set pivot??
                RobotState.getInstance().setRobotCurrentAction(RobotCurrentAction.kAutoShootAtPosition);
                RobotState.getInstance().actualAutoShootAtPositionPose = AllianceFlipUtil.apply(AutoScanLoop.shoot(notes[currentNote]));

                // If within range of pose
                //      Shoot
                //      Next Note in Queue
                //      Generate path to next note in queue
                break;
            case ENDING_DRIVING:
                // Drive to Next Note Pos? Prolly unnecessary
                // Actuate and start intake
                // Start Shooter / Set pivot
                RobotState.getInstance().getDrive().setProfile(DriveProfiles.kAutoPiecePickup);
                if(isHoldingNote()){
                    Pose2d closest = getVisibleNoteLocation();
                    RobotState.getInstance().getDrive().setDriveTurnOverride(RobotState.getInstance().getShooterMath().setNextShootingPoseAndVelocity(RobotState.getInstance().getEstimatedPose(),new Twist2d(),new Translation3d(closest.getX(),closest.getY(),0)).get(0));
                }



                // If we have note
                //      set state to shooting
                //      turn towards shooter??
                break;
            case ENDING_SHOOTING:
                // Turn to shooter
                // Run Shooter / Pivot
                RobotState.getInstance().setRobotCurrentAction(RobotCurrentAction.kAutoShootAtPosition);
                RobotState.getInstance().actualAutoShootAtPositionPose = AllianceFlipUtil.apply(AutoScanLoop.shoot(endNotes[currentNote]));
                // If within shooting tolerance
                //      Shoot
                //      Next Note in Queue
                //      Generate path to next note in queue
                
            
        
            default:
                break;
        }

        // Path Generation
    }

    public void shot() {
        notesShot++;
        if(currAutoState == AutoState.SHOOTING) {
            currAutoState = AutoState.SCANNING;
            mDriveToPiece = RobotState.getInstance().getAutoFactory().generateTrajectoryToPose(AllianceFlipUtil.apply(getNextNotePosition(false)), DriveConstants.kDriveToPieceSpeed, false, RobotState.getInstance());
            mDriveToPiece.schedule();
        }else{
            currAutoState = AutoState.ENDING_DRIVING;
            mDriveToPiece = RobotState.getInstance().getAutoFactory().generateTrajectoryToPose(AllianceFlipUtil.apply(getNextNotePosition(true)), DriveConstants.kDriveToPieceSpeed, false, RobotState.getInstance());
            mDriveToPiece.schedule();
        }
    }

    public enum AutoState {
        STARTING,
        SCANNING,
        SHOOTING,
        DRIVING_TO_NOTE,
        ENDING_DRIVING,
        ENDING_SHOOTING,
        ENDING
    }

    public void reset(){
        currAutoState = AutoState.STARTING;
        currentNote = -1;
        notesShot = 0;

 
     }

    public boolean isCameraWorking() {
        return false;
    }

   public boolean isNoteDetected() {
        if (RobotState.getInstance().getClosestNote() != null) {
            return true;
        }
        return false;
   }

   public Pose2d getVisibleNoteLocation() {
        // first get all poses
        // find the one that is closest to the current note we are looking for
        Pose2d poses = RobotState.getInstance().getClosestNote();
        return poses;
        // closest note to the note we actually want
        // map each note based on what note they are closest to
        //  get closest note
        

        // HashMap<Integer, Note> noteMap = new HashMap<>();

        // for (Pose2d pose : poses) {

        //     Note closestNote = AutoScanLoop.getClosestNote(pose);
        //     if (noteMap.containsKey(closestNote.getId())) {
        //         if (pose.getTranslation().getDistance(closestNote.getPosition()) < noteMap.get(closestNote.getId()).getPosition().getDistance(closestNote.getPosition())) {
        //             noteMap.put(closestNote.getId(), new Note(closestNote.getId(), pose.getTranslation()));
        //         }
        //     } else {
        //         noteMap.put(closestNote.getId(), new Note(closestNote.getId(), pose.getTranslation()));
        //     }


        // }
        // return null;
   }

   public Pose2d getNextNotePosition(boolean isEnding) {
        currentNote++;
        Logger.recordOutput("AutoBuilderManager/notesShot", notesShot);
        Logger.recordOutput("AutoBuilderManager/notesLength", notes.length);
        Logger.recordOutput("AutoBuilderManager/num_scouts", num_scouts);
        if (!isEnding) {
            if (currentNote < notes.length && notesShot < num_scouts) {
                currAutoState = AutoState.SCANNING;
                return new Pose2d(notes[currentNote].getPosition(), new Rotation2d());
            }
            currentNote = 0;
            currAutoState = AutoState.ENDING_DRIVING;
        }

        if (currentNote < endNotes.length) {
            currAutoState = AutoState.ENDING_DRIVING;
            return new Pose2d(endNotes[currentNote].getPosition(), new Rotation2d());
        }

        currAutoState = AutoState.ENDING;
        return endingPose;
   }

   public boolean isHoldingNote() {
        return RobotState.getInstance().hasNote();
   }

}
