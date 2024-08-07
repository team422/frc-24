// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

/** Generic roller IO implementation for a roller or series of rollers using a SPARK Flex. */
public abstract class GenericRollerSystemIOSparkFlex implements GenericRollerSystemIO {
  private final CANSparkFlex motor;
  private final RelativeEncoder encoder;

  private final double reduction;

  public GenericRollerSystemIOSparkFlex(
      int id, int currentLimitAmps, boolean invert, boolean brake, double reduction) {
    this.reduction = reduction;
    motor = new CANSparkFlex(id, CANSparkBase.MotorType.kBrushless);

    motor.setSmartCurrentLimit(currentLimitAmps);
    motor.setInverted(invert);
    motor.setIdleMode(brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);

    encoder = motor.getEncoder();
  }

  public void updateInputs(GenericRollerSystemIOInputs inputs) {
    inputs.positionRads = Units.rotationsToRadians(encoder.getPosition()) / reduction;
    inputs.velocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / reduction;
    inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.outputCurrent = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void runVolts(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
