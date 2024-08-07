package frc.robot.subsystems.shooter.pivot;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ShooterConstants.ShooterPivotConstants;

public class PivotIOSim implements PivotIO {


    public Rotation2d m_desiredAngle;

    public SingleJointedArmSim m_armSim;    

    public PIDController  m_Controller;

    public PivotIOSim() {
        m_armSim = new SingleJointedArmSim(DCMotor.getFalcon500Foc(1),ShooterPivotConstants.gearboxRatio,10, .13,Rotation2d.fromDegrees(-20).getRadians(),ShooterPivotConstants.maxAngle.getRadians(),false,ShooterPivotConstants.homeAngle.getRadians()); 
        m_desiredAngle = Rotation2d.fromDegrees(30);
        m_Controller = new PIDController(10.7,0,0);
    }

    @Override
    public void clearI(){
        m_Controller.setIntegratorRange(-0.3,0.3);
    }
    @Override
    public void runSetpoint(Rotation2d angle, double feedforward) {
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

    @Override
    public Rotation2d getDesiredAngle() {
        return m_desiredAngle;
    }

    @Override
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRadians(m_armSim.getAngleRads());
    }

    @Override
    public double getCurrentVelocity() {
        return m_armSim.getVelocityRadPerSec();
    }
    
}
