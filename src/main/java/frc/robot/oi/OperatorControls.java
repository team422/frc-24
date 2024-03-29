package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorControls {

    public Trigger setClimbTop();

    public Trigger setClimbBottom();

    public Trigger climbUp();

    public Trigger climbDown();

}
