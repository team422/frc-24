package frc.robot.subsystems.objectVision;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.utils.GeomUtil;
import frc.lib.utils.VirtualSubsystem;
import frc.robot.Constants.Vision;
import frc.robot.Constants.Vision.ObjectDetection;
import frc.robot.subsystems.objectVision.ObjectDetectionIO.ObjectDetectionVisionIOInputs;

public class ObjectDetectionCam extends VirtualSubsystem {

      private final ObjectDetectionIO[] io;
  private final ObjectDetectionVisionIOInputs[] inputs;

  private final Map<Integer, Double> lastFrameTimes = new HashMap<>();

  private ArrayList<Pose2d> positions = new ArrayList<Pose2d>();
  private ArrayList<Pose2d> allPoses = new ArrayList<Pose2d>();

  private Integer loses = 0;



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

          allPoses.clear();
          for (int i = 0; i <io.length; i++){
            lastFrameTimes.put(i, Timer.getFPGATimestamp());
            ArrayList<Transform3d> noteTransforms = new ArrayList<Transform3d>();
            var values = inputs[i].objects;
            Logger.recordOutput("Number of values", values.length);
            
            if (values.length == 0) {
                positions.clear();
              }
                for (int j = 0; j < values.length; j++) {
                    double[] object = values[j];
                    if (object.length % 4 != 0) {
                        continue;
                    }
                    // object is a double array of length a multiple of 7
                    // each 7 elements represent the x, y, z, tW, tX, tY, tZ of a detected object
                    ArrayList<Double[]> objectList = new ArrayList<Double[]>();
                    for (int k = 0; k < object.length; k+=4) {
                        Double[] objectArray = {object[k], object[k+1], object[k+2], object[k+3]};
                        objectList.add(objectArray);
                    }
                    // if(positions.size() > objectList.size()){
                    //   if(loses > 25){
                    //     positions.clear();
                    //   } else {
                    //     loses +=1;
                    //   }
                    // }else{
                    //   loses = 0;
                    // }
                    positions.clear();
                    for (Double[] objectArray : objectList) {
                        // double x = objectArray[0];
                        // double y = objectArray[1];
                        // double z = objectArray[2];
                        // double tW = objectArray[3];
                        // double tX = objectArray[4];
                        // double tY = objectArray[5];
                        // double tZ = objectArray[6];
                        double xmin = objectArray[0];
                        double xmax = objectArray[1];
                        double ymin = objectArray[2];
                        double ymax = objectArray[3];
                        double xavg = 2*((xmin+xmax)/2)-1;
                        // double[] translation = {x, y, z};
                        double h = Units.inchesToMeters(26.413);
                        double minDepressionAngle = Units.degreesToRadians(Vision.ObjectDetection.angleMin.get());
                        double maxDepressionAngle = Units.degreesToRadians(Vision.ObjectDetection.angleDepression.get());
                        double thetaM = Units.degreesToRadians(j);
                        double thetam = Units.degreesToRadians(j);
                        // double[] rotation = {tW, tX, tY, tZ};
                        double x = h * Math.tan(maxDepressionAngle - (ymin+ymax/2)*(maxDepressionAngle - minDepressionAngle));
                        double bottomXAcross = 1/2;
                        double topXAcross = 3.0/2;
                        double y = -xavg*(ObjectDetection.distanceFar.get()-((ymin+ymax)/2)*(ObjectDetection.distanceFar.get()-ObjectDetection.distanceClose.get()));

                        Logger.recordOutput("NoteXDistance",x);
                        Logger.recordOutput("NoteYDistance",y);
                        // Transform3d noteTranslation = new Transform3d(new Translation3d(translation[0], translation[1], translation[2]), new Rotation3d(new Quaternion(0,0,0,0)));

                        // noteTransforms.add(noteTranslation);

                        // add current position to positions array
                        Pose3d currentPose = new Pose3d(frc.robot.RobotState.getInstance().getPoseTimeAgo(0.2));
                        currentPose = currentPose.transformBy(GeomUtil.pose3dToTransform3d(ObjectDetectionVisionConstants.cameraPoses[i]));
                        Logger.recordOutput("Camera Pose",currentPose);
                        Pose2d twoDimensionalPose = new Pose2d(currentPose.getX(),currentPose.getY(),Rotation2d.fromRadians(currentPose.getRotation().getAngle()));
                        Transform2d noteTranslation = new Transform2d(x,y,new Rotation2d());

                        

                        
                        Pose2d fPose = new Pose2d(twoDimensionalPose.plus(noteTranslation).getTranslation(),new Rotation2d());
                        positions.add(fPose);
                    }
                    allPoses.addAll(positions);
                }

                // log the positions
                Logger.recordOutput("ObjectDetectionVision/Inst" , allPoses.toArray(Pose2d[]::new));
          }


          // if (Robot.isSimulation()){
          //   HashMap<String,Pose2d> val = frc.robot.FieldConstants.getGamePieces();
          //   for (Map.Entry<String, Pose2d> entry : val.entrySet()) {
          //       String key = entry.getKey();
          //       Pose2d value = entry.getValue();
          //       positions.add(new Pose3d(value.getTranslation().getX(), value.getTranslation().getY(), 0.0, new Rotation3d(0.0,0.0,value.getRotation().getRadians()))); 
          //   }
          // }






        
    }

    public Pose2d getClosestNote() {
        if (allPoses.size() == 0) {
            return null;
        }
        else {
            Pose2d currentPose = frc.robot.RobotState.getInstance().getEstimatedPose();
            Pose2d closestNote = new Pose2d();
            double closestDistance = Double.MAX_VALUE;
            for (Pose2d note : allPoses) {
                double distance = note.getTranslation().getDistance(currentPose.getTranslation());
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestNote = new Pose2d(note.getTranslation().getX(), note.getTranslation().getY(), note.getRotation());
                }
            }
            return closestNote;
    }

   
  }
  public Pose2d[] getAllNotes(){
    Pose2d[] notes = new Pose2d[allPoses.size()];
    for (int i = 0; i < allPoses.size(); i++) {
        Pose2d note = allPoses.get(i);
        notes[i] = new Pose2d(note.getTranslation().getX(), note.getTranslation().getY(), note.getRotation());
    }
    return notes;
}
    
}
