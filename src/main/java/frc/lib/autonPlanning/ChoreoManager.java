package frc.lib.autonPlanning;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Collections;
import java.util.List;
import java.util.stream.Stream;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ChoreoManager {
    
    public static List<String> getExistingPaths() {
        var path = Path.of(Filesystem.getDeployDirectory().getAbsolutePath(), "choreo");
        try (Stream<Path> stream = Files.walk(path)) {
          // return Collections.emptyList();
        //   make it so the end is .traj and the filename does not include .1.
        // do not include anything after the . in the filename
            return stream
                .filter(Files::isRegularFile)
                .map(Path::toFile)
                .filter(file -> getFileExtension(file).equals(".traj"))
                .map(File::getName)
                .filter(name -> !name.contains(".1."))
                .map(name -> name.substring(0, name.indexOf(".")))
                .toList();
        } catch (IOException e) {
          return Collections.emptyList();
        }
      }
      private static String getFileExtension(File file) {
        try {
          String name = file.getName();
          return name.substring(name.lastIndexOf("."));
        } catch (Exception e) {
          return "";
        }
      }
}

