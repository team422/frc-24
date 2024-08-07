package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drive.ctre.SwerveDrivetrain;
import frc.robot.subsystems.drive.ctre.SwerveModule;
import frc.robot.subsystems.drive.ctre.SwerveRequest;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.001; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }


    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {

        return run(() -> this.setControl(requestSupplier.get()));
    }
    public void applyRequest(SwerveRequest request) {

         this.setControl(request);
    }

    public SwerveModule[] getModules() {
        return Modules;

    }

    public SwerveModuleState[] getTargetStates(){
        SwerveModuleState[] m_modules = new SwerveModuleState[4];
        for(int i = 0; i<=3; i++){
            m_modules[i] = Modules[i].getTargetState();
        }
        return m_modules;
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] m_modules = new SwerveModuleState[4];
        for(int i = 0; i<=3; i++){
            m_modules[i] = Modules[i].getCurrentState();
        }
        return m_modules;
    }


    private void startSimThread() {

        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, 12);
        });
        // m_simNotifier.startPeriodic(kSimLoopPeriod);
        // m_simNotifier.setName("Drive Periodic");
    }
}
