package frc.robot;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.lib.utils.SwerveTester;
import frc.robot.Constants.Vision.AprilTagVision;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.objectVision.ObjectDetectionCam;
import frc.robot.subsystems.shooter.Shooter;

public class RobotState {
    // singleton class that handles all inter subsystem communication
    private static RobotState instance = null;
    private Drive m_drive;
    private Climb m_climb;
    private Indexer m_indexer;
    private Shooter m_shooter;
    private ObjectDetectionCam[] m_objectDetectionCams;
    private AprilTagVision[] m_aprilTagVisions;
    private Intake m_intake;

    private SwerveTester m_swerveTester;
    private RobotState(Drive drive, Climb climb, Indexer indexer, Shooter shooter, ObjectDetectionCam[] objectDetectionCams, AprilTagVision[] aprilTagVisions, Intake intake) {
        this.m_drive = drive;
        this.m_climb = climb;
        this.m_shooter = shooter;
        this.m_indexer = indexer;
        this.m_objectDetectionCams = objectDetectionCams;
        this.m_aprilTagVisions = aprilTagVisions;
        this.m_intake = intake;
        // this.m_swerveTester = new SwerveTester(drive);
    }
    public static RobotState startInstance(Drive drive, Climb climb, Indexer indexer, Shooter shooter, ObjectDetectionCam[] objectDetectionCams, AprilTagVision[] aprilTagVisions, Intake intake) {
        if (instance == null) {
            instance = new RobotState(drive, climb, indexer,shooter, objectDetectionCams, aprilTagVisions, intake);
        }
        return instance;
    }

   
    public static RobotState getInstance() {
        
        return instance;
    }

    public void updateRobotState() {
        // update all subsystems
        Pose2d curPose = m_drive.getPose();
        Transform3d curTransformIntake = m_intake.getTransform();
        Logger.recordOutput("RobotState/IntakePose", curTransformIntake);
        Transform3d curTransformShooter = m_shooter.getTransform();
        Logger.recordOutput("RobotState/ShooterPose", curTransformShooter);


    }

    
// under construction
     public SwerveTester getSwerveTester() {
        return m_swerveTester;
    }

    public ArrayList<AprilTag> getAprilTagsInView() {
        return new ArrayList<AprilTag>();
    }
    public void setAprilTagMap(AprilTagFieldLayout aprilTags) {
        // set the april tag map
    }
}
