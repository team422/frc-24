package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;

public enum Note {
    ONE(FieldConstants.StagingLocations.spikeTranslations[0]),
    TWO(FieldConstants.StagingLocations.spikeTranslations[1]),
    THREE(FieldConstants.StagingLocations.spikeTranslations[2]),
    FOUR(FieldConstants.StagingLocations.centerlineTranslations[4]),
    FIVE(FieldConstants.StagingLocations.centerlineTranslations[3]),
    SIX(FieldConstants.StagingLocations.centerlineTranslations[2]), 
    SEVEN(FieldConstants.StagingLocations.centerlineTranslations[1]), 
    EIGHT(FieldConstants.StagingLocations.centerlineTranslations[0]);
    
    private Translation2d position;

    public Translation2d getPosition() {
        return position;
    }

    Note(Translation2d position) {
        this.position = position;
    }
}