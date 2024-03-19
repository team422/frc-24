// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.RobotState;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.Supplier;
import java.util.random.RandomGenerator;
import java.util.stream.Stream;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class NoteVisualizer {
  private static final double shotSpeed = 9.0; // Meters per sec
  private static final double ejectSpeed = 2.0; // Meters per sec
  @Setter private static Supplier<Pose2d> robotPoseSupplier = Pose2d::new;
  @Setter private static Supplier<Rotation2d> armAngleSupplier = Rotation2d::new;
  public static final List<Translation2d> autoNotes = new ArrayList<>();
  public static final List<Translation2d> fieldNotes = new ArrayList<>();
  @Setter private static boolean hasNote = false;

  private static boolean isShooting = false;

  public static final Timer timer = new Timer();

  /** Show all staged notes for alliance */
  public static void showAllNotes() {
    if (autoNotes.isEmpty()) {
      Logger.recordOutput("NoteVisualizer/StagedNotes");
    }
    // Show auto notes
    Stream<Translation2d> presentNotes = autoNotes.stream().filter(Objects::nonNull);
    // add field notes
    presentNotes = Stream.concat(presentNotes, fieldNotes.stream());
    timer.start();
    Logger.recordOutput(
        "NoteVisualizer/StagedNotes",
        presentNotes
            .map(
                translation ->
                    new Pose3d(
                        translation.getX(),
                        translation.getY(),
                        Units.inchesToMeters(1.0),
                        new Rotation3d()))
            .toArray(Pose3d[]::new));
  }

  public static void clearAutoNotes() {
    autoNotes.clear();
  }

  /** Add all notes to be shown at the beginning of auto */
  public static void resetAutoNotes() {
    clearAutoNotes();
    for (int i = frc.robot.FieldConstants.StagingLocations.spikeTranslations.length - 1; i >= 0; i--) {
      autoNotes.add(AllianceFlipUtil.apply(frc.robot.FieldConstants.StagingLocations.spikeTranslations[i]));
    }
    for (int i =frc.robot.FieldConstants.StagingLocations.centerlineTranslations.length - 1; i >= 0; i--) {
      autoNotes.add(
          AllianceFlipUtil.apply(frc.robot.FieldConstants.StagingLocations.centerlineTranslations[i]));
    }
  }

  /**
   * Take note from staged note
   *
   * @param note Number of note starting with 0 - 2 being spike notes going from amp to source side
   *     <br>
   *     and 3 - 7 being centerline notes going from amp to source side.
   */
  public static void takeAutoNote(int note) {
    autoNotes.set(note, null);
    hasNote = true;
  }

  public static List<Translation2d> getAutoNotes() {
    return autoNotes;
  }

  public static List<Translation2d> getAllNotes() {
    List<Translation2d> allNotes = new ArrayList<>();
    allNotes.addAll(autoNotes.stream().filter(Objects::nonNull).toList());
    allNotes.addAll(fieldNotes);
    return allNotes;
  }

  public static void takeNote(Translation2d note) {
    fieldNotes.remove(note);
    hasNote = true;
  }

  /** Shows the currently held note if there is one */
  public static void showHeldNotes() {
    if (hasNote) {
      Logger.recordOutput("NoteVisualizer/HeldNotes", new Pose3d[] {getIndexerPose3d()});
    } else {
      Logger.recordOutput("NoteVisualizer/HeldNotes", new Pose3d());
    }
  }


  /** Shoots note from middle of arm to speaker */
  public static Command shoot() {
    if(isShooting || hasNote == false){
      return Commands.run(() -> {});
    }
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  hasNote = false;
                  isShooting = true;
                  final Pose3d startPose = getIndexerPose3d();
                  final Pose3d endPose =
                      new Pose3d(
                          AllianceFlipUtil.apply(frc.robot.FieldConstants.centerSpeakerOpening),
                          startPose.getRotation());

                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation()) / shotSpeed;
                  final Timer timer = new Timer();
                  timer.start();
                  return Commands.run(
                          () ->
                              Logger.recordOutput(
                                  "NoteVisualizer/ShotNotes",
                                  new Pose3d[] {
                                    startPose.interpolate(endPose, timer.get() / duration)
                                  }))
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(() -> {Logger.recordOutput("NoteVisualizer/ShotNotes");
                      isShooting = false;
                      });
                },
                Set.of())
            .ignoringDisable(true));
  }

  public static Command eject() {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  hasNote = false;
                  final Pose3d startPose = getIndexerPose3d();
                  System.out.println(getIndexerPose3d().getZ());
                  final Pose3d endPose =
                      startPose.transformBy(
                          new Transform3d(2, 0, -1 + startPose.getZ(), new Rotation3d()));

                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation()) / ejectSpeed;
                  final Timer timer = new Timer();
                  timer.start();
                  return Commands.run(
                          () ->
                              Logger.recordOutput(
                                  "NoteVisualizer/ShotNotes",
                                  new Pose3d[] {
                                    startPose.interpolate(endPose, timer.get() / duration)
                                  }))
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(() -> Logger.recordOutput("NoteVisualizer/EjectNotes"));
                },
                Set.of())
            .ignoringDisable(true));
  }

  private static Pose3d getIndexerPose3d() {
    // Transform3d indexerTransform =
    //     new Transform3d(
    //             ArmConstants.armOrigin.getX(),
    //             0.0,
    //             ArmConstants.armOrigin.getY(),
    //             new Rotation3d(0.0, -armAngleSupplier.get().getRadians(), 0.0))
    //         .plus(new Transform3d(ArmConstants.armLength * 0.35, 0.0, 0.0, new Rotation3d()));
    Transform3d indexerTransform = RobotState.getInstance().getShooterTransform();
    return new Pose3d(robotPoseSupplier.get()).plus(indexerTransform);
  }

  private static void addNoteToSource() {
    
      Translation2d sourceNote = AllianceFlipUtil.apply(frc.robot.FieldConstants.Source.source);
      sourceNote = sourceNote.plus(new Translation2d(RandomGenerator.getDefault().nextDouble(-1.0, 0), RandomGenerator.getDefault().nextDouble(0, 1)));
      fieldNotes.add(sourceNote);
      // add random translation next to source 
      


  }

  public static void checkWhetherToAddNote(){
    Pose2d pose = robotPoseSupplier.get();
    // check distance to source
    if (pose.getTranslation().getDistance(frc.robot.FieldConstants.Source.source) < 4.0 && !hasNote && timer.get() > 3.0) {
      addNoteToSource();
      timer.reset();
    }



  }
}
