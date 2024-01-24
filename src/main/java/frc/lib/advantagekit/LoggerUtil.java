package frc.lib.advantagekit;

import java.io.File;
import java.nio.file.Path;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import frc.robot.BuildConstants;
// import frc.robot.BuildConstants;
import frc.robot.Robot;

public class LoggerUtil {
  public static final String kLogDirectory = "advantagekit/logs";

  public static void initializeLogger() {
    Logger logger = Logger.getInstance();

    // Record metadata
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    logger.recordMetadata("GitDirty", getGitDirtyString());

    // Set up data receivers & replay source
    if (Robot.isSimulation()) {
      logger.addDataReceiver(new WPILOGWriter(getLogPath()));
      logger.addDataReceiver(new NT4Publisher());
    } else {
      logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
      logger.addDataReceiver(new NT4Publisher());
    }

    // Start AdvantageKit logger
    logger.start();
  }

  private static String getGitDirtyString() {
    switch (BuildConstants.DIRTY) {
      case 0:
        return "All changes committed";
      case 1:
        return "Uncomitted changes";
      default:
        return "Unknown";
    }
  }

  private static String getLogPath() {
    File file = Path.of(kLogDirectory).toFile();

    if (!file.exists()) {
      file.mkdirs();
    }

    return file.getAbsolutePath();
  }
}
