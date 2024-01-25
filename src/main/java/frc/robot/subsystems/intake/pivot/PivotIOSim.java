package frc.robot.subsystems.intake.pivot;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim implements PivotIO {
    
    private PIDController m_controller;

    private SingleJointedArmSim m_armSim;

    private Rotation2d m_desiredAngle;



    public PivotIOSim() {

        m_armSim = new SingleJointedArmSim(DCMotor.getNEO(1), 24.2391, SingleJointedArmSim.estimateMOI(0.21, 4), 0.21, Rotation2d.fromDegrees(-30).getRadians(), Rotation2d.fromDegrees(110).getRadians(),true, Rotation2d.fromDegrees(0).getDegrees());

        m_controller = new PIDController(8.7, 0, 0);

        m_desiredAngle =  Rotation2d.fromRadians(m_armSim.getAngleRads());
        
    }

    @Override
    public void setDesiredAngle(Rotation2d angle) {
       m_desiredAngle = angle; 
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        m_armSim.update(0.02);
        inputs.curAngle = (m_armSim.getAngleRads());
        inputs.curSpeed = m_armSim.getVelocityRadPerSec();
        double output = m_controller.calculate(inputs.curAngle, m_desiredAngle.getRadians());
        inputs.voltage = output;
        // System.out.println("Desired: "+ output);
        // System.out.println(inputs.curAngle);
        m_armSim.setInputVoltage(output);
        
    }
}
