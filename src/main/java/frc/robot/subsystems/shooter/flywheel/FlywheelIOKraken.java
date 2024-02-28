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

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants.FlywheelConstants;

public class FlywheelIOKraken implements FlywheelIO {

    TalonFX m_krakenLeft;
    TalonFX m_krakenRight;

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
      new VelocityTorqueCurrentFOC(0.0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);

  private  DCMotorSim m_simLeft = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 1.0/2, 0.005);
    private  DCMotorSim m_simRight = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 1.0/2, 0.005);




    public FlywheelIOKraken(int motorID1, int motorID2) {
        m_krakenLeft = new TalonFX(motorID1, "rio");
        m_krakenRight = new TalonFX(motorID2, "rio");
      


        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 50.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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
        m_krakenLeft.getConfigurator().apply(config, 1.0);
        m_krakenLeft.getConfigurator().apply(controllerConfig, 1.0);
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_krakenRight.getConfigurator().apply(config,1.0);
        m_krakenRight.getConfigurator().apply(controllerConfig,1.0);
        // m_krakenSecond.setControl(new Follower(motorID1, true));

        // Set signals
        Position = m_krakenLeft.getPosition();
        Velocity = m_krakenLeft.getVelocity();
        AppliedVolts = m_krakenLeft.getMotorVoltage();
        TorqueCurrent = m_krakenLeft.getTorqueCurrent();
        TempCelsius = m_krakenLeft.getDeviceTemp();

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
        m_krakenLeft.setControl(velocityControl.withVelocity(rpm/60.0));
        m_krakenRight.setControl(velocityControl.withVelocity((rpm*0.95)/60.0));

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
            inputs.secondMotorConnected = m_krakenRight.isAlive();

            inputs.PositionRads = Units.rotationsToRadians(Position.getValueAsDouble());
            inputs.VelocityRpm = Velocity.getValueAsDouble() * 60.0;
            inputs.AppliedVolts = AppliedVolts.getValueAsDouble();
            inputs.OutputCurrent = TorqueCurrent.getValueAsDouble();
            inputs.TempCelsius = TempCelsius.getValueAsDouble();

            simulationPeriodic();

    }

    public void simulationPeriodic(){
        TalonFXSimState leftSim = m_krakenLeft.getSimState();
        TalonFXSimState rightSim = m_krakenRight.getSimState();
        m_simLeft.setInputVoltage(leftSim.getMotorVoltage());
        m_simRight.setInputVoltage(rightSim.getMotorVoltage());

        m_simLeft.update(0.02);
        m_simRight.update(0.02);
        m_krakenLeft.getSimState().setRawRotorPosition(m_simLeft.getAngularPositionRotations());
        m_krakenRight.getSimState().setRawRotorPosition(m_simRight.getAngularPositionRotations());
        
        m_krakenLeft.getSimState().setRotorVelocity(m_simLeft.getAngularVelocityRPM()/60.0);
        m_krakenRight.getSimState().setRotorVelocity(m_simRight.getAngularVelocityRPM()/60.0);        

    }

     @Override
  public void stop() {
    m_krakenLeft.setControl(neutralControl);
  }
}
