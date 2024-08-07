package frc.robot.subsystems.shooter.flywheel;

import java.util.ArrayList;

import javax.print.attribute.standard.MediaSize.Other;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.utils.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants.FlywheelConstants;
import frc.robot.subsystems.shooter.Shooter.ShooterProfile;
import frc.robot.utils.CtreBaseRefreshManager;

public class FlywheelIOKraken implements FlywheelIO {

    TalonFX m_krakenLeft;
    TalonFX m_krakenRight;

    // Status Signals
    private final StatusSignal<Double> Position;
    private final StatusSignal<Double> Velocity;
    private final StatusSignal<Double> AppliedVolts;
    private final StatusSignal<Double> TorqueCurrent;
    private final StatusSignal<Double> TempCelsius;
    private final StatusSignal<Double> PositionLeft;
    private final StatusSignal<Double> VelocityLeft;
    private final StatusSignal<Double> AppliedVoltsLeft;
    private final StatusSignal<Double> TorqueCurrentLeft;
    private final StatusSignal<Double> TempCelsiusLeft;
    private final StatusSignal<Double> OutputCurrentLeft;
    private final StatusSignal<Double> OutputCurrent;

    // Control
    private final Slot0Configs controllerConfigLeft = new Slot0Configs();
    private final Slot0Configs controllerConfigRight = new Slot0Configs();
    private final VoltageOut voltageControl = new VoltageOut(0.0).withUpdateFreqHz(0.0);
    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
    private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0.0);
    private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);
    private final PIDController mController = new PIDController(1, 0, 0);
    private final PIDController mControllerLeft = new PIDController(1, 0, 0);
    private SimpleMotorFeedforward mFeedforwardLeft = new SimpleMotorFeedforward(1, 0, 0);
    private SimpleMotorFeedforward mFeedforwardRight = new SimpleMotorFeedforward(1, 0, 0);
    private DCMotorSim m_simLeft = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 1.0 / 2.0, 0.001);
    private DCMotorSim m_simRight = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 1.0 / 2.0, .001);
    TalonFXConfiguration config;

    private Enum<?> lastProfile;

    public FlywheelIOKraken(int motorID1, int motorID2) {
        m_krakenLeft = new TalonFX(motorID1, "rio");
        m_krakenRight = new TalonFX(motorID2, "rio");

        config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimit = 65.0;
        config.CurrentLimits.StatorCurrentLimit = 150.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = Constants.ShooterConstants.FlywheelConstants.kFlywheelGearRatio;

        // Controller config;
        controllerConfigLeft.kP = FlywheelConstants.kFlywheelP.get();
        controllerConfigLeft.kI = FlywheelConstants.kFlywheelI.get();
        controllerConfigLeft.kD = FlywheelConstants.kFlywheelD.get();
        controllerConfigLeft.kS = FlywheelConstants.kFlywheelKSLeft.get();
        controllerConfigLeft.kV = FlywheelConstants.kFlywheelKVLeft.get();
        controllerConfigLeft.kA = FlywheelConstants.kFlywheelKALeft.get();

        controllerConfigRight.kP = FlywheelConstants.kFlywheelP.get();
        controllerConfigRight.kI = FlywheelConstants.kFlywheelI.get();
        controllerConfigRight.kD = FlywheelConstants.kFlywheelD.get();
        controllerConfigRight.kS = FlywheelConstants.kFlywheelKSRight.get();
        controllerConfigRight.kV = FlywheelConstants.kFlywheelKVRight.get();
        controllerConfigRight.kA = FlywheelConstants.kFlywheelKARight.get();

        // Apply configs
        m_krakenLeft.getConfigurator().apply(config, 1.0);
        m_krakenLeft.getConfigurator().apply(controllerConfigLeft, 1.0);
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_krakenRight.getConfigurator().apply(config, 1.0);
        m_krakenRight.getConfigurator().apply(controllerConfigRight, 1.0);
        // m_krakenSecond.setControl(new Follower(motorID1, true));

        // Set signals
        Position = m_krakenLeft.getPosition();
        Velocity = m_krakenRight.getVelocity();
        // VelocityLeft = m_krakenRight.getVelocity();
        AppliedVolts = m_krakenLeft.getMotorVoltage();

        TorqueCurrent = m_krakenLeft.getTorqueCurrent();
        TempCelsius = m_krakenRight.getDeviceTemp();
        PositionLeft = m_krakenRight.getPosition();
        VelocityLeft = m_krakenLeft.getVelocity();
        AppliedVoltsLeft = m_krakenRight.getMotorVoltage();
        TorqueCurrentLeft = m_krakenRight.getTorqueCurrent();
        TempCelsiusLeft = m_krakenLeft.getDeviceTemp();
        OutputCurrentLeft = m_krakenLeft.getSupplyCurrent();
        OutputCurrent = m_krakenRight.getSupplyCurrent();
        

        BaseStatusSignal.setUpdateFrequencyForAll(
                250.0,
                Position,
                Velocity,
                AppliedVolts,
                TorqueCurrent,
                TempCelsius,
                PositionLeft,
                VelocityLeft,
                PositionLeft,
                TorqueCurrentLeft,
                TempCelsiusLeft);

                ArrayList<StatusSignal> signals = new ArrayList<StatusSignal>();
                signals.add(Position);
                signals.add(Velocity);
                signals.add(AppliedVolts);
                signals.add(TorqueCurrent);
                signals.add(TempCelsius);
                signals.add(PositionLeft);
                signals.add(VelocityLeft);
                signals.add(PositionLeft);
                signals.add(TorqueCurrentLeft);
                signals.add(TempCelsiusLeft);
                signals.add(OutputCurrent);
                signals.add(OutputCurrentLeft);
                signals.add(AppliedVolts);
                signals.add(AppliedVoltsLeft);
                CtreBaseRefreshManager.getInstance().addSignals(signals);

    }


 
    public double metersPerSecondToRPM(double metersPerSecond) {
        double diameter = FlywheelConstants.kFlywheelDiameter;
        double circumference = Math.PI * diameter;
        double rotationsPerSecond = metersPerSecond / circumference;
        double rpm = rotationsPerSecond * 60.0;
        return rpm;

    }

    @Override
    public void setProfile(Enum<?> profile) {

        if (lastProfile == null) {
            lastProfile = profile;
        } else if (profile != lastProfile) {
            lastProfile = profile;
            if (profile.equals(ShooterProfile.activeShooting)) {

            } else if (profile.equals(ShooterProfile.notReving)) {
                m_krakenLeft.stopMotor();
                m_krakenRight.stopMotor();
            } else if (profile.equals(ShooterProfile.slowRev)) {

            }
        }

    }

    public double RPMtoMetersPerSecond(double RPM) {
        double diameter = FlywheelConstants.kFlywheelDiameter;
        double circumference = Math.PI * diameter;
        // double rotationsPerSecond = metersPerSecond / circumference;
        double metersPerSecond = (RPM / 60) * circumference;
        return metersPerSecond;

    }

    @Override
    public void setDesiredSpeedWithSpin(double speedLeft, double speedRight) {
        // Logger.recordOutput("Speed Right PID
        // ",mControllerLeft.calculate(metersPerSecondToRPM(speedLeft)/60.0,
        // m_krakenLeft.getRotorVelocity().getValueAsDouble()));
        Logger.recordOutput("Speed Left Velocity", m_krakenLeft.getRotorVelocity().getValueAsDouble());
        Logger.recordOutput("Speed Left Acceleration", m_krakenLeft.getAcceleration().getValueAsDouble());
        Logger.recordOutput("Speed Left desired", metersPerSecondToRPM(speedLeft) / 60.0);
        Logger.recordOutput("Speed Left Feedforward Voltage",
                mFeedforwardLeft.calculate(metersPerSecondToRPM(speedLeft) / 60.0));
        // Logger.recordOutput("Speed Left PID
        // ",mControllerLeft.calculate(metersPerSecondToRPM(speedLeft)/60.0,
        // m_krakenRight.getRotorVelocity().getValueAsDouble()));
        Logger.recordOutput("Speed Right Velocity", m_krakenRight.getRotorVelocity().getValueAsDouble());
        Logger.recordOutput("Speed Right Acceleration", m_krakenRight.getAcceleration().getValueAsDouble());
        Logger.recordOutput("Speed Right desired", metersPerSecondToRPM(speedRight) / 60.0);
        Logger.recordOutput("Speed Right Feedforward Voltage",
                mFeedforwardRight.calculate(metersPerSecondToRPM(speedLeft) / 60.0));

        // m_krakenLeft.setControl(velocityControl.withVelocity(metersPerSecondToRPM(speedLeft)/60.0));
        // m_krakenRight.setControl(velocityControl.withVelocity((metersPerSecondToRPM(speedRight))/60.0));
        // m_krakenRight.setControl(new TorqueCurrentFOC(15));
        if (lastProfile.equals(ShooterProfile.slowRev)) {
            if (Robot.isReal()) {
                m_krakenRight
                        .setControl(new VoltageOut(mFeedforwardRight.calculate(metersPerSecondToRPM(speedRight) / 60.0))
                                .withUpdateFreqHz(0.0));
                m_krakenLeft
                        .setControl(new VoltageOut(mFeedforwardLeft.calculate(metersPerSecondToRPM(speedLeft) / 60.0))
                                .withUpdateFreqHz(0.0));
            }

        } else if (lastProfile.equals(ShooterProfile.activeShooting)) {
            if (Robot.isReal()) {
                m_krakenRight.setControl(new VoltageOut(mFeedforwardRight.calculate(
                        metersPerSecondToRPM(speedRight) / 60.0,
                        ((m_krakenRight.getRotorVelocity().getValueAsDouble()
                                - metersPerSecondToRPM(speedRight) / 60.0)) * FlywheelConstants.kFlywheelAccel.get())
                        + mController.calculate(metersPerSecondToRPM(speedRight) / 60.0,
                                m_krakenRight.getRotorVelocity().getValueAsDouble())));
                m_krakenLeft.setControl(new VoltageOut(mFeedforwardLeft.calculate(
                        metersPerSecondToRPM(speedLeft) / 60.0,
                        ((m_krakenLeft.getRotorVelocity().getValueAsDouble() - metersPerSecondToRPM(speedLeft) / 60.0))
                                * FlywheelConstants.kFlywheelAccel.get())
                        + mControllerLeft.calculate(metersPerSecondToRPM(speedLeft) / 60.0,
                                m_krakenLeft.getRotorVelocity().getValueAsDouble())));
            } else {

                m_krakenRight.setControl(new VoltageOut(mController.calculate(metersPerSecondToRPM(speedRight) / 60.0,
                        m_krakenRight.getRotorVelocity().getValueAsDouble())));
                m_krakenLeft.setControl(new VoltageOut(mControllerLeft.calculate(metersPerSecondToRPM(speedLeft) / 60.0,
                        m_krakenLeft.getRotorVelocity().getValueAsDouble())));
            }
        }

    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            // controllerConfig.kP = FlywheelConstants.kFlywheelP.get();
            // controllerConfig.kI = FlywheelConstants.kFlywheelI.get();
            // controllerConfig.kD = FlywheelConstants.kFlywheelD.get();
            // controllerConfig.kS = FlywheelConstants.kFlywheelKS.get();
            // controllerConfig.kV = FlywheelConstants.kFlywheelKV.get();
            // controllerConfig.kA = FlywheelConstants.kFlywheelKA.get();
            if (Robot.isReal()) {
                mControllerLeft.setPID(FlywheelConstants.kFlywheelP.get(), FlywheelConstants.kFlywheelI.get(),
                        FlywheelConstants.kFlywheelD.get());
                mController.setPID(FlywheelConstants.kFlywheelP.get(), FlywheelConstants.kFlywheelI.get(),
                        FlywheelConstants.kFlywheelD.get());
            } else {
                mControllerLeft.setPID(FlywheelConstants.kFlywheelP.get(), FlywheelConstants.kFlywheelISIM.get(),
                        FlywheelConstants.kFlywheelD.get());
                mController.setPID(FlywheelConstants.kFlywheelP.get(), FlywheelConstants.kFlywheelISIM.get(),
                        FlywheelConstants.kFlywheelD.get());
            }
            mFeedforwardLeft = new SimpleMotorFeedforward(FlywheelConstants.kFlywheelKSLeft.get(),
                    FlywheelConstants.kFlywheelKVLeft.get(), FlywheelConstants.kFlywheelKALeft.get());
            mFeedforwardRight = new SimpleMotorFeedforward(FlywheelConstants.kFlywheelKSRight.get(),
                    FlywheelConstants.kFlywheelKVRight.get(), FlywheelConstants.kFlywheelKARight.get());

            m_krakenLeft.getConfigurator().apply(controllerConfigLeft, 1.0);
            m_krakenRight.getConfigurator().apply(controllerConfigRight, 1.0);
        }, FlywheelConstants.kFlywheelP, FlywheelConstants.kFlywheelISIM, FlywheelConstants.kFlywheelI,
                FlywheelConstants.kFlywheelD, FlywheelConstants.kFlywheelKARight, FlywheelConstants.kFlywheelKSRight,
                FlywheelConstants.kFlywheelKVRight, FlywheelConstants.kFlywheelKVLeft,
                FlywheelConstants.kFlywheelKSLeft, FlywheelConstants.kFlywheelKALeft);
        // inputs.firstMotorConnected = BaseStatusSignal.refreshAll(
        //         Position,
        //         Velocity,
        //         AppliedVolts,
        //         TorqueCurrent,
        //         TempCelsius,
        //         PositionLeft,
        //         VelocityLeft,
        //         PositionLeft,
        //         TorqueCurrentLeft,
        //         OutputCurrent,
        //         OutputCurrentLeft,
        //         TempCelsiusLeft, AppliedVolts, AppliedVoltsLeft)
        //         .isOK();
        inputs.secondMotorConnected = m_krakenRight.isAlive();

        inputs.PositionRads = Units.rotationsToRadians(Position.getValueAsDouble());
        inputs.VelocityRpm = VelocityLeft.getValueAsDouble() * 60.0;
        inputs.AppliedVolts = AppliedVolts.getValueAsDouble();
        inputs.OutputCurrent = OutputCurrent.getValueAsDouble();
        inputs.TempCelsius = TempCelsius.getValueAsDouble();
        inputs.PositionRadsLeft = Units.rotationsToRadians(PositionLeft.getValueAsDouble());
        inputs.VelocityRpmLeft = Velocity.getValueAsDouble() * 60.0;
        inputs.VelocityLeft = VelocityLeft.getValueAsDouble();
        inputs.Velocity = Velocity.getValueAsDouble();
        inputs.AppliedVoltsLeft = AppliedVolts.getValueAsDouble();
        inputs.OutputCurrentLeft = OutputCurrentLeft.getValueAsDouble();
        inputs.TempCelsiusLeft = TempCelsiusLeft.getValueAsDouble();
        inputs.AppliedVoltsLeft = AppliedVoltsLeft.getValueAsDouble();
        if (Robot.isSimulation()) {
            simulationPeriodic();
        }

    }

    public void simulationPeriodic() {
        TalonFXSimState leftSim = m_krakenLeft.getSimState();
        TalonFXSimState rightSim = m_krakenRight.getSimState();
        m_simLeft.setInputVoltage(leftSim.getMotorVoltage());
        m_simRight.setInputVoltage(rightSim.getMotorVoltage());

        m_simLeft.update(0.02);
        m_simRight.update(0.02);
        m_krakenLeft.getSimState().setRawRotorPosition(m_simLeft.getAngularPositionRotations());
        m_krakenRight.getSimState().setRawRotorPosition(m_simRight.getAngularPositionRotations());

        m_krakenLeft.getSimState().setRotorVelocity(m_simLeft.getAngularVelocityRPM() / 60.0);
        m_krakenRight.getSimState().setRotorVelocity(m_simRight.getAngularVelocityRPM() / 60.0);

    }

    @Override
    public void stop() {
        m_krakenLeft.setControl(neutralControl);
    }

    @Override
    public boolean checkIfTolerance(double left, double right, double tolerance) {
        // Logger.recordOutput("Delta",
        // m_krakenLeft.getRotorVelocity().getValueAsDouble() - left);
        // Logger.recordOutput("Delta Right",
        // m_krakenRight.getRotorVelocity().getValueAsDouble() - right);
        return Math.abs(m_krakenLeft.getRotorVelocity().getValueAsDouble() - left) < tolerance
                && Math.abs(m_krakenRight.getRotorVelocity().getValueAsDouble() - right) < tolerance;

    }

    @Override
    public void setFlywheelCurrentLimit(double limit) {
        config.CurrentLimits.SupplyCurrentLimit = limit;
        m_krakenLeft.getConfigurator().apply(config.CurrentLimits, 1.0);
        m_krakenRight.getConfigurator().apply(config.CurrentLimits, 1.0);

    }
}
