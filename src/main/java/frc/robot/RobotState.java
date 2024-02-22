package frc.robot;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.lib.hardwareprofiler.HardwareProfiler;
import frc.lib.hardwareprofiler.ProfilingScheduling;
import frc.lib.utils.SwerveTester;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants.ShooterPivotConstants;
import frc.robot.commands.autonomous.CustomTrajectoryRunner;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.northstarAprilTagVision.AprilTagVision;
import frc.robot.subsystems.objectVision.ObjectDetectionCam;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.ShooterMath;

public class RobotState {
    // singleton class that handles all inter subsystem communication
    private static RobotState instance = null;
    private Drive m_drive;
    private Climb m_climb;
    private Indexer m_indexer;
    private Shooter m_shooter;
    private ObjectDetectionCam[] m_objectDetectionCams;
    private AprilTagVision m_aprilTagVision;
    private Intake m_intake;
    private ShooterMath m_shooterMath;
    private SwerveTester m_swerveTester;
    private CustomTrajectoryRunner m_customTrajectoryRunner;


    public enum GamePieceLocation {
        INTAKE,
        INDEXER,
        SHOOTER,
        NOT_IN_ROBOT
    }

    private GamePieceLocation m_gamePieceLocation = GamePieceLocation.NOT_IN_ROBOT;

    private RobotState(Drive drive, Climb climb, Indexer indexer, Shooter shooter, ObjectDetectionCam[] objectDetectionCams, AprilTagVision aprilTagVisions, Intake intake) {
        this.m_drive = drive;
        this.m_climb = climb;
        this.m_shooter = shooter;
        this.m_indexer = indexer;
        this.m_objectDetectionCams = objectDetectionCams;
        this.m_aprilTagVision = aprilTagVisions;
        this.m_intake = intake;
        m_shooterMath = new ShooterMath(16.496);
        ProfilingScheduling.startInstance();
        
        m_customTrajectoryRunner = new CustomTrajectoryRunner();
        // this.m_swerveTester = new SwerveTester(drive);
    }
    public static RobotState startInstance(Drive drive, Climb climb, Indexer indexer, Shooter shooter, ObjectDetectionCam[] objectDetectionCams, AprilTagVision aprilTagVision, Intake intake) {
        if (instance == null) {
            
            instance = new RobotState(drive, climb, indexer,shooter, objectDetectionCams, aprilTagVision, intake);

        }
        return instance;
    }



   
    public static RobotState getInstance() {
        
        return instance;
    }


    public Rotation2d getMaxIntakeAngle() {
        // first get desired angle of the shooter
        Rotation2d shooterAngle = m_shooter.getPivotAngle();
        if (shooterAngle.getDegrees() < 20) {
            return IntakeConstants.kIntakeMaxMovedAngle;
        }
        return IntakeConstants.kIntakeMaxAngle;
    }


    public void setGamePieceLocation(GamePieceLocation location) {
        m_gamePieceLocation = location;
        if (location == GamePieceLocation.INDEXER) {
            m_indexer.setState(Indexer.IndexerState.INDEXING);
        }
    }

    public Pose2d getRobotPose() {
        return m_drive.getPose();
    }

    public Pose2d getDesiredRobotPose(){
        // return m_drive.getDesiredPose();
        return new Pose2d();
    }

    public void updateTestScheduler(){
        if (ProfilingScheduling.hasTests() && !DriverStation.isFMSAttached()) {
            ProfilingScheduling.updateCurrent();
        }
    }

    public void updateRobotState() {
        

        // update all subsystems
        // Pose2d curPose = m_drive.getPose();
        // Transform3d curTransformIntake = m_intake.getTransform();
        // Logger.recordOutput("RobotState/IntakePose", curTransformIntake);
        // Transform3d curTransformShooter = m_shooter.getTransform();
        // Logger.recordOutput("RobotState/ShooterPose", curTransformShooter);


    }

    

    public void calculateShooterAngle() {
        // calculate the shooter angle
        ArrayList<Rotation2d> angles = m_shooterMath.getShooterAngle(new Pose3d(FieldConstants.kShooterCenter,new Rotation3d()),new Pose3d(m_drive.getPose()));
        m_shooter.setPivotAngle(angles.get(1));
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
