package frc.robot.subsystems.shooter.pivot;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ShooterConstants.PivotConstants;

public class PivotIOSim implements PivotIO {


    public Rotation2d m_desiredAngle;

    public SingleJointedArmSim m_armSim;    

    public PIDController  m_Controller;

    public PivotIOSim() {
        m_armSim = new SingleJointedArmSim(DCMotor.getFalcon500Foc(1),PivotConstants.gearboxRatio,10, .13,Rotation2d.fromDegrees(0).getRadians(),PivotConstants.maxAngle.getRadians(),false,PivotConstants.homeAngle.getRadians()); 
        m_desiredAngle = Rotation2d.fromDegrees(30);
        m_Controller = new PIDController(.7,0,0);
    }

    @Override
    public void setDesiredAngle(Rotation2d angle) {
        m_desiredAngle = angle;
    }


    @Override
    public void updateInputs(PivotIOInputs m_inputs) {
        m_armSim.update(0.02);
        m_inputs.curAngle = (m_armSim.getAngleRads());
        m_inputs.curSpeed = m_armSim.getVelocityRadPerSec();
        double output = m_Controller.calculate(m_inputs.curAngle, m_desiredAngle.getRadians());
        m_inputs.voltage = output;
        m_armSim.setInputVoltage(output);
    }
    
}
