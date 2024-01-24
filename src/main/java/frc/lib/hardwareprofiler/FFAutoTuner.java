package frc.lib.hardwareprofiler;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class FFAutoTuner {
  boolean m_gTuned;
  boolean m_vTuned;
  boolean m_aTuned;
  boolean m_sTuned;
  double m_kG;
  double m_kV;
  double m_kA;
  double m_kS;
  double m_kVMax;
  double m_kAMax;
  double m_kSMax;
  double m_kGMax;
  double m_kVMin;
  double m_kAMin;
  double m_kSMin;
  double m_kGMin;

  public enum TuningMode {
    kG,
    kV,
    kA,
    kS
  }

  TuningMode m_tuningMode;

  public enum FFType {
    ArmFeedforward,
    ElevatorFeedforward,
    SimpleMotorFeedforward,
  }

  FFType m_ffType;

  public enum currentRunning {
    kResetting,
    kRunning,
    kFinished
  }

  double[] m_positionSetpointRange = new double[2];
  double[] m_velocitySetpointRange = new double[2];
  double[] m_accelerationSetpointRange = new double[2];
  double m_velocityStep;
  double m_accelerationStep;
  String m_name;
  double m_cSetpoint;

  SimpleMotorFeedforward m_simpleMotorFeedforward;
  ArmFeedforward m_armFeedforward;
  ElevatorFeedforward m_elevatorFeedforward;

  public FFAutoTuner(TuningMode tuningMode, FFType ffType, double[] positionSetpointRange,
      double[] velocitySetpointRange, double[] accelerationSetpointRange, double positionStep, double velocityStep,
      double accelerationStep, String name) {
    m_gTuned = false;
    m_vTuned = false;
    m_aTuned = false;
    m_sTuned = false;
    m_kG = 0;
    m_kV = 0;
    m_kA = 0;
    m_kS = 0;
    m_kVMax = 3;
    m_kAMax = 3;
    m_kSMax = 3;
    m_kGMax = 3;
    m_kVMin = 0;
    m_kAMin = 0;
    m_kSMin = 0;
    m_kGMin = 0;
    m_tuningMode = tuningMode;
    m_ffType = ffType;
    m_positionSetpointRange = positionSetpointRange;
    m_velocitySetpointRange = velocitySetpointRange;
    m_accelerationSetpointRange = accelerationSetpointRange;
    m_velocityStep = velocityStep;
    m_accelerationStep = accelerationStep;
    m_name = name;
    m_cSetpoint = positionSetpointRange[0];
  }

  public double calculateBinarySearch(double min, double max) {
    return (max + min) / 2;
  }

  public void setTuningMode(TuningMode tuningMode) {
    m_tuningMode = tuningMode;
  }

  public void setFFType(FFType ffType) {
    m_ffType = ffType;
  }

  public void setMax(double max) {
    switch (m_tuningMode) {
      case kG:
        m_kGMax = max;
        break;
      case kV:
        m_kVMax = max;
        break;
      case kA:
        m_kAMax = max;
        break;
      case kS:
        m_kSMax = max;
        break;
    }
  }

  public void setMin(double min) {
    switch (m_tuningMode) {
      case kG:
        m_kGMin = min;
        break;
      case kV:
        m_kVMin = min;
        break;
      case kA:
        m_kAMin = min;
        break;
      case kS:
        m_kSMin = min;
        break;
    }
  }

  public void setTuned(boolean tuned) {
    switch (m_tuningMode) {
      case kG:
        m_gTuned = tuned;
        break;
      case kV:
        m_vTuned = tuned;
        break;
      case kA:
        m_aTuned = tuned;
        break;
      case kS:
        m_sTuned = tuned;
        break;
    }
  }

  public boolean getTuned() {
    switch (m_tuningMode) {
      case kG:
        return m_gTuned;
      case kV:
        return m_vTuned;
      case kA:
        return m_aTuned;
      case kS:
        return m_sTuned;
      default:
        return false;
    }
  }

}
