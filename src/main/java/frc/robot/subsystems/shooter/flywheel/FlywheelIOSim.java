package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {
    public FlywheelIOInputs inputs = new FlywheelIOInputs();
    public PIDController m_Controller;
    FlywheelSim m_FlywheelSim;
    public FlywheelIOSim() {
        super();
        m_FlywheelSim = new FlywheelSim(DCMotor.getKrakenX60Foc(2), 1/1.5, 1.0);
        m_Controller = new PIDController(1,0,0);

    }

    // @Override
    // public void setDesiredSpeed(double speed) {
    //     inputs.desiredSpeed = speed;
    //     double output = m_Controller.calculate(inputs.curSpeed, inputs.desiredSpeed);
    //     inputs.voltage = output;
    //     m_FlywheelSim.setInputVoltage(output);
    //     m_FlywheelSim.update(0.02);
    // }



    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        this.inputs = inputs;
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        inputs.desiredSpeed = 0;
    }

    
    
}
