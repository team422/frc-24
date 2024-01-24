// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.northstarAprilTagVision;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.utils.Alert;
import frc.lib.utils.Alert.AlertType;
import frc.robot.FieldConstants;

public class AprilTagVisionIONorthstar implements AprilTagVisionIO {
  private static final int cameraId = 0;
  private static final int cameraResolutionWidth = 1280;
  private static final int cameraResolutionHeight = 720;
  private static final int cameraAutoExposure = 1;
  private static final int cameraExposure = 5;
  private static final int cameraGain = 25;

  private final DoubleArraySubscriber observationSubscriber;
  private final IntegerSubscriber fpsSubscriber;

  private static final double disconnectedTimeout = 0.5;
  private final Alert disconnectedAlert;
  private final Timer disconnectedTimer = new Timer();

  public AprilTagVisionIONorthstar(String identifier) {
    var northstarTable = NetworkTableInstance.getDefault().getTable(identifier);

    var configTable = northstarTable.getSubTable("config");
    configTable.getIntegerTopic("camera_id").publish().set(cameraId);
    configTable.getIntegerTopic("camera_resolution_width").publish().set(cameraResolutionWidth);
    configTable.getIntegerTopic("camera_resolution_height").publish().set(cameraResolutionHeight);
    configTable.getIntegerTopic("camera_auto_exposure").publish().set(cameraAutoExposure);
    configTable.getIntegerTopic("camera_exposure").publish().set(cameraExposure);
    configTable.getIntegerTopic("camera_gain").publish().set(cameraGain);
    configTable.getDoubleTopic("fiducial_size_m").publish().set(FieldConstants.aprilTagWidth);
    try {
      // String jsonString = "{\"tags\":[{\"ID\":1,\"pose\":{\"translation\":{\"x\":15.079471999999997,\"y\":0.24587199999999998,\"z\":1.355852},\"rotation\":{\"quaternion\":{\"W\":0.5000000000000001,\"X\":0.0,\"Y\":0.0,\"Z\":0.8660254037844386}}}},{\"ID\":2,\"pose\":{\"translation\":{\"x\":16.185134,\"y\":0.883666,\"z\":1.355852},\"rotation\":{\"quaternion\":{\"W\":0.5000000000000001,\"X\":0.0,\"Y\":0.0,\"Z\":0.8660254037844386}}}},{\"ID\":3,\"pose\":{\"translation\":{\"x\":16.579342,\"y\":4.982717999999999,\"z\":1.4511020000000001},\"rotation\":{\"quaternion\":{\"W\":6.123233995736766E-17,\"X\":0.0,\"Y\":0.0,\"Z\":1.0}}}},{\"ID\":4,\"pose\":{\"translation\":{\"x\":16.579342,\"y\":5.547867999999999,\"z\":1.4511020000000001},\"rotation\":{\"quaternion\":{\"W\":6.123233995736766E-17,\"X\":0.0,\"Y\":0.0,\"Z\":1.0}}}},{\"ID\":5,\"pose\":{\"translation\":{\"x\":14.700757999999999,\"y\":8.2042,\"z\":1.355852},\"rotation\":{\"quaternion\":{\"W\":-0.7071067811865475,\"X\":-0.0,\"Y\":0.0,\"Z\":0.7071067811865476}}}},{\"ID\":6,\"pose\":{\"translation\":{\"x\":1.8415,\"y\":8.2042,\"z\":1.355852},\"rotation\":{\"quaternion\":{\"W\":-0.7071067811865475,\"X\":-0.0,\"Y\":0.0,\"Z\":0.7071067811865476}}}},{\"ID\":7,\"pose\":{\"translation\":{\"x\":-0.038099999999999995,\"y\":5.547867999999999,\"z\":1.4511020000000001},\"rotation\":{\"quaternion\":{\"W\":1.0,\"X\":0.0,\"Y\":0.0,\"Z\":0.0}}}},{\"ID\":8,\"pose\":{\"translation\":{\"x\":-0.038099999999999995,\"y\":4.982717999999999,\"z\":1.4511020000000001},\"rotation\":{\"quaternion\":{\"W\":1.0,\"X\":0.0,\"Y\":0.0,\"Z\":0.0}}}},{\"ID\":9,\"pose\":{\"translation\":{\"x\":0.356108,\"y\":0.883666,\"z\":1.355852},\"rotation\":{\"quaternion\":{\"W\":0.8660254037844387,\"X\":0.0,\"Y\":0.0,\"Z\":0.49999999999999994}}}},{\"ID\":10,\"pose\":{\"translation\":{\"x\":1.4615159999999998,\"y\":0.24587199999999998,\"z\":1.355852},\"rotation\":{\"quaternion\":{\"W\":0.8660254037844387,\"X\":0.0,\"Y\":0.0,\"Z\":0.49999999999999994}}}},{\"ID\":11,\"pose\":{\"translation\":{\"x\":11.904726,\"y\":3.7132259999999997,\"z\":1.3208},\"rotation\":{\"quaternion\":{\"W\":-0.8660254037844387,\"X\":-0.0,\"Y\":0.0,\"Z\":0.49999999999999994}}}},{\"ID\":12,\"pose\":{\"translation\":{\"x\":11.904726,\"y\":4.49834,\"z\":1.3208},\"rotation\":{\"quaternion\":{\"W\":0.8660254037844387,\"X\":0.0,\"Y\":0.0,\"Z\":0.49999999999999994}}}},{\"ID\":13,\"pose\":{\"translation\":{\"x\":11.220196,\"y\":4.105148,\"z\":1.3208},\"rotation\":{\"quaternion\":{\"W\":6.123233995736766E-17,\"X\":0.0,\"Y\":0.0,\"Z\":1.0}}}},{\"ID\":14,\"pose\":{\"translation\":{\"x\":5.320792,\"y\":4.105148,\"z\":1.3208},\"rotation\":{\"quaternion\":{\"W\":1.0,\"X\":0.0,\"Y\":0.0,\"Z\":0.0}}}},{\"ID\":15,\"pose\":{\"translation\":{\"x\":4.641342,\"y\":4.49834,\"z\":1.3208},\"rotation\":{\"quaternion\":{\"W\":0.5000000000000001,\"X\":0.0,\"Y\":0.0,\"Z\":0.8660254037844386}}}},{\"ID\":16,\"pose\":{\"translation\":{\"x\":4.641342,\"y\":3.7132259999999997,\"z\":1.3208},\"rotation\":{\"quaternion\":{\"W\":-0.49999999999999983,\"X\":-0.0,\"Y\":0.0,\"Z\":0.8660254037844388}}}}],\"field\":{\"length\":16.451,\"width\":8.211}}";

      configTable
          .getStringTopic("tag_layout")
          .publish()
          .set(new ObjectMapper().writeValueAsString(FieldConstants.getAprilTags()));
      // .set(jsonString);

    } catch (JsonProcessingException e) {
      throw new RuntimeException("Failed to serialize AprilTag layout JSON for Northstar");
    }

    var outputTable = northstarTable.getSubTable("output");
    observationSubscriber = outputTable
        .getDoubleArrayTopic("observations")
        .subscribe(
            new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    fpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0);

    disconnectedAlert = new Alert("No data from \"" + identifier + "\"", AlertType.ERROR);
    disconnectedTimer.start();
  }

  public void updateInputs(AprilTagVisionIOInputs inputs) {
    var queue = observationSubscriber.readQueue();
    inputs.timestamps = new double[queue.length];
    inputs.frames = new double[queue.length][];
    for (int i = 0; i < queue.length; i++) {
      inputs.timestamps[i] = queue[i].timestamp / 1000000.0;
      inputs.frames[i] = queue[i].value;
    }
    inputs.fps = fpsSubscriber.get();

    // Update disconnected alert
    if (queue.length > 0) {
      disconnectedTimer.reset();
    }
    disconnectedAlert.set(disconnectedTimer.hasElapsed(disconnectedTimeout));
  }
}
