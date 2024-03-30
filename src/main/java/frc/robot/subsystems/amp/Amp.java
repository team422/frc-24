package frc.robot.subsystems.amp;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import frc.lib.hardwareprofiler.ProfiledSubsystem;
import frc.lib.utils.LoggedTunableNumber;
import frc.robot.Constants.AmpConstants;

public class Amp extends ProfiledSubsystem {
    AmpIO m_ampIO;
    AmpIOInputsAutoLogged m_inputsPivot;


    Rotation2d ampRotation2d;

    public enum AmpState {
        PositionFollowing,Zeroing
    }

    public AmpState m_state;


    public Amp(AmpIO ampIO) {
        m_ampIO = ampIO;
        ampRotation2d = ampIO.getPosition();
    }

    



    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void periodic() {
        LoggedTunableNumber.ifChanged(
      hashCode(), () -> {
        m_ampIO.setPID(AmpConstants.kAmpP.get(),AmpConstants.kAmpI.get(),AmpConstants.kAmpD.get());
        }, AmpConstants.kAmpP,AmpConstants.kAmpI,AmpConstants.kAmpD);
        if (RobotState.isAutonomous()){
            return;
        }
        
        m_ampIO.updateInputs(m_inputsPivot);
        
        
        Logger.processInputs("Amp Pivot", m_inputsPivot);
        if(m_state == AmpState.PositionFollowing){
            m_ampIO.setPosition(ampRotation2d);

        }
        else{
            m_ampIO.zero();
        }

        
    }

    public void setPivotAngle(Rotation2d angle) {
        // if (Robot.isSimulation()) {
        //     angle = angle.minus(ShooterPivotConstants.simOffset);
        // }
        // System.out.println("Setting angle to: "+angle.getDegrees());
        ampRotation2d = angle;
        
    }

    

    

    public Rotation2d getPivotAngle() {
        return m_ampIO.getPosition();
    }

    
}
