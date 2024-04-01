package frc.robot.utils;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.robot.Constants.AutoRuns;

public class AutoScanLoop {




    public AutoScanLoop() {
        
    }



    public static boolean isAtScanThreshold(Note note, Pose2d pose) {

        return note.getPosition().getDistance(pose.getTranslation()) < 2;
        
        
    }

    public static Pose2d shoot(Note note) {
        return null;
    }

    public static Note getClosestNote(Pose2d pose) {
        Note closestNote = null;
        double closestDistance = Double.MAX_VALUE;
        Note[] notes = Note.values();
        for (Note note : notes) {
            double distance = note.getPosition().getDistance(pose.getTranslation());
            if (distance < closestDistance) {
                closestDistance = distance;
                closestNote = note;
            }
        }
        return closestNote;
    }
    
    public HashMap<String,Note[]> getAllScoutPossibilties(){
        return AutoRuns.scoutPossibilties;
    }

    public HashMap<String,Note[]> getAllEndPossibilties(){
        return AutoRuns.endPossibilties;
    }

    public HashMap<String,Integer> getScoutNumberPossibilties(){
        return AutoRuns.scoutNumberPossibilities;
    }
}