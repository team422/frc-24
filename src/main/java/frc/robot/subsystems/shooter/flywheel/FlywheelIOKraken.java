package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants.FlywheelConstants;

public class FlywheelIOKraken implements FlywheelIO {

    TalonFX m_krakenFirst;
    TalonFX m_krakenSecond;

  // Status Signals
  private final StatusSignal<Double> Position;
  private final StatusSignal<Double> Velocity;
  private final StatusSignal<Double> AppliedVolts;
  private final StatusSignal<Double> TorqueCurrent;
  private final StatusSignal<Double> TempCelsius;


  // Control
  private final Slot0Configs controllerConfig = new Slot0Configs();
  private final VoltageOut voltageControl = new VoltageOut(0.0).withUpdateFreqHz(0.0);
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VelocityTorqueCurrentFOC velocityControl =
      new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);


    public FlywheelIOKraken(int motorID1, int motorID2) {
        m_krakenFirst = new TalonFX(motorID1);
        m_krakenSecond = new TalonFX(motorID2);

        m_krakenFirst.getSimState();        


        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Feedback.SensorToMechanismRatio = Constants.ShooterConstants.FlywheelConstants.kFlywheelGearRatio;

        // Controller config;
        controllerConfig.kP = FlywheelConstants.kFlywheelP.get();
        controllerConfig.kI = FlywheelConstants.kFlywheelI.get();
        controllerConfig.kD = FlywheelConstants.kFlywheelD.get();
        controllerConfig.kS = FlywheelConstants.kFlywheelKS.get();
        controllerConfig.kV = FlywheelConstants.kFlywheelKV.get();
        controllerConfig.kA = FlywheelConstants.kFlywheelKA.get();

        // Apply configs
        m_krakenFirst.getConfigurator().apply(config, 1.0);
        m_krakenFirst.getConfigurator().apply(controllerConfig, 1.0);
        m_krakenSecond.setControl(new Follower(motorID1, true));

        // Set signals
        Position = m_krakenFirst.getPosition();
        Velocity = m_krakenFirst.getVelocity();
        AppliedVolts = m_krakenFirst.getMotorVoltage();
        TorqueCurrent = m_krakenFirst.getTorqueCurrent();
        TempCelsius = m_krakenFirst.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            Position,
            Velocity,
            AppliedVolts,
            TorqueCurrent,
            TempCelsius
            );

    }


    public double metersPerSecondToRPM(double metersPerSecond) {
        double diameter = FlywheelConstants.kFlywheelDiameter;
        double circumference = Math.PI * diameter;
        double rotationsPerSecond = metersPerSecond / circumference;
        double rpm = rotationsPerSecond * 60.0;
        return rpm;

    }

    @Override
    public void setDesiredSpeed(double speed) {
        double rpm = metersPerSecondToRPM(speed);
        m_krakenFirst.setControl(velocityControl.withVelocity(rpm/60.0));

    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
            inputs.firstMotorConnected =
            BaseStatusSignal.refreshAll(
                Position,
                Velocity,
                AppliedVolts,
                TorqueCurrent,
                TempCelsius)
                    .isOK();
            inputs.secondMotorConnected = m_krakenSecond.isAlive();

            inputs.PositionRads = Units.rotationsToRadians(Position.getValueAsDouble());
            inputs.VelocityRpm = Velocity.getValueAsDouble() * 60.0;
            inputs.AppliedVolts = AppliedVolts.getValueAsDouble();
            inputs.OutputCurrent = TorqueCurrent.getValueAsDouble();
            inputs.TempCelsius = TempCelsius.getValueAsDouble();
    }
     @Override
  public void stop() {
    m_krakenFirst.setControl(neutralControl);
  }
}
