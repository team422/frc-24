package frc.robot.subsystems.objectVision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.utils.VirtualSubsystem;
import frc.robot.Robot;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.objectVision.ObjectDetectionIO.ObjectDetectionVisionIOInputs;

public class ObjectDetectionCam extends VirtualSubsystem {

      private final ObjectDetectionIO[] io;
  private final ObjectDetectionVisionIOInputs[] inputs;

  private final Map<Integer, Double> lastFrameTimes = new HashMap<>();

  private ArrayList<Pose3d> positions = new ArrayList<Pose3d>();



  public ObjectDetectionCam(ObjectDetectionIO... io) {
    this.io = io;
    inputs = new ObjectDetectionVisionIOInputs[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new ObjectDetectionVisionIOInputs();
    }

    // Create map of last frame times for instances
    for (int i = 0; i < io.length; i++) {
      lastFrameTimes.put(i, 0.0);
    }

  }
    @Override
    public void periodic() {

        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("ObjectDetectionVision/Inst" + i, inputs[i]);
          }


          for (int i = 0; i <io.length; i++){
            lastFrameTimes.put(i, Timer.getFPGATimestamp());
            ArrayList<Transform3d> noteTransforms = new ArrayList<Transform3d>();
            var values = inputs[i].objects;
            if (values.length == 0 || values[0].length == 1) {
                continue;
              }
                for (int j = 0; j < values.length; j++) {
                    double[] object = values[j];
                    if (object.length % 7 != 0) {
                        continue;
                    }
                    // object is a double array of length a multiple of 7
                    // each 7 elements represent the x, y, z, tW, tX, tY, tZ of a detected object
                    ArrayList<Double[]> objectList = new ArrayList<Double[]>();
                    for (int k = 0; k < object.length; k+=7) {
                        Double[] objectArray = {object[k], object[k+1], object[k+2], object[k+3], object[k+4], object[k+5], object[k+6]};
                        objectList.add(objectArray);
                    }
                    for (Double[] objectArray : objectList) {
                        double x = objectArray[0];
                        double y = objectArray[1];
                        double z = objectArray[2];
                        double tW = objectArray[3];
                        double tX = objectArray[4];
                        double tY = objectArray[5];
                        double tZ = objectArray[6];
                        double[] translation = {x, y, z};
                        double[] rotation = {tW, tX, tY, tZ};

                        Transform3d noteTranslation = new Transform3d(new Translation3d(translation[0], translation[1], translation[2]), new Rotation3d(new Quaternion(tW, tX, tY, tZ)));

                        noteTransforms.add(noteTranslation);

                        // add current position to positions array
                        Pose3d currentPose = new Pose3d(frc.robot.RobotState.getInstance().getEstimatedPose());

                        positions.clear();

                        
                        Pose3d fPose = new Pose3d(currentPose.plus(noteTranslation).getTranslation(),currentPose.plus(noteTranslation).getRotation().plus(new Rotation3d(0,0,Math.PI)));
                        positions.add(fPose);
                    }
                }

                // log the positions
                Logger.recordOutput("ObjectDetectionVision/Inst" , positions.get(0));
          }


          if (Robot.isSimulation()){
            HashMap<String,Pose2d> val = frc.robot.FieldConstants.getGamePieces();
            for (Map.Entry<String, Pose2d> entry : val.entrySet()) {
                String key = entry.getKey();
                Pose2d value = entry.getValue();
                positions.add(new Pose3d(value.getTranslation().getX(), value.getTranslation().getY(), 0.0, new Rotation3d(0.0,0.0,value.getRotation().getRadians()))); 
            }
          }






        
    }

    public Pose2d getClosestNote() {
        if (positions.size() == 0) {
            return null;
        }
        else {
            Pose2d currentPose = frc.robot.RobotState.getInstance().getEstimatedPose();
            Pose2d closestNote = new Pose2d();
            double closestDistance = Double.MAX_VALUE;
            for (Pose3d note : positions) {
                double distance = note.getTranslation().toTranslation2d().getDistance(currentPose.getTranslation());
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestNote = new Pose2d(note.getTranslation().getX(), note.getTranslation().getY(), note.getRotation().toRotation2d());
                }
            }
            return closestNote;
    }
  }
    
}
