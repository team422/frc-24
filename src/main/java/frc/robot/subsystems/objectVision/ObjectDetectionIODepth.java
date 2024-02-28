package frc.robot.subsystems.objectVision;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import frc.lib.utils.Alert;

public class ObjectDetectionIODepth implements ObjectDetectionIO {
  private static final int cameraResolutionWidth = 1600;
  private static final int cameraResolutionHeight = 1200;
  private static final int cameraAutoExposure = 1;
  private static final int cameraExposure = 10;
  private static final int cameraGain = 25;

  private final DoubleArraySubscriber observationSubscriber;
  private final IntegerSubscriber fpsSubscriber;

  private static final double disconnectedTimeout = 0.5;
//   private final Alert disconnectedAlert;

  public ObjectDetectionIODepth(String instanceId) {
    var northstarTable = NetworkTableInstance.getDefault().getTable("ObjectDetectionCamera");

    // var configTable = northstarTable.getSubTable("config");
    // configTable.getStringTopic("camera_id").publish().set(cameraId);
    // configTable.getIntegerTopic("camera_resolution_width").publish().set(cameraResolutionWidth);
    // configTable.getIntegerTopic("camera_resolution_height").publish().set(cameraResolutionHeight);
    // configTable.getIntegerTopic("camera_auto_exposure").publish().set(cameraAutoExposure);
    // configTable.getIntegerTopic("camera_exposure").publish().set(cameraExposure);
    // configTable.getIntegerTopic("camera_gain").publish().set(cameraGain);
    // configTable.getDoubleTopic("fiducial_size_m").publish().set(frc.robot.FieldConstants.aprilTagWidth);
    // try {
    //   configTable
    //       .getStringTopic("tag_layout")
    //       .publish()
    //       .set(new ObjectMapper().writeValueAsString(frc.robot.FieldConstants.getAprilTags()));
    // } catch (JsonProcessingException e) {
    //   throw new RuntimeException("Failed to serialize AprilTag layout JSON for Northstar");
    // }

    var outputTable = northstarTable.getSubTable("output");
    observationSubscriber =
        outputTable
            .getDoubleArrayTopic("observations")
            .subscribe(
                new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    fpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0);

    // disconnectedAlert = new Alert("No data from \"" + instanceId + "\"", Alert.AlertType.ERROR);
    // disconnectedTimer.start();
  }

  public void updateInputs(ObjectDetectionVisionIOInputs inputs) {
    var queue = observationSubscriber.readQueue();
    inputs.timestamps = new double[queue.length];
    inputs.objects = new double[queue.length][];
    System.out.println(queue.length);
    for (int i = 0; i < queue.length; i++) {
        inputs.timestamps[i] = queue[i].timestamp / 1000000.0;
        inputs.objects[i] = queue[i].value;
    }
    inputs.fps = fpsSubscriber.get();
    // System.out.println(inputs.fps);

    // Update disconnected alert
    // if (queue.length > 0) {
    //   disconnectedTimer.reset();
    // }
    // disconnectedAlert.set(disconnectedTimer.hasElapsed(disconnectedTimeout));
  }

    
}
