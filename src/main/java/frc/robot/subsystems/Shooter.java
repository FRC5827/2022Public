// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import frc.robot.Constants.General;

public class Shooter extends SubsystemBase {

  private WPI_TalonFX lower = createTalon(ShooterConstants.kLowerDeviceID);
  private WPI_TalonFX upper = createTalon(ShooterConstants.kUpperDeviceID);

  private double m_upperVelocitySetpoint;
  private double m_lowerVelocitySetpoint;

  private FlywheelSim lowerWheelSim = new FlywheelSim(
      DCMotor.getFalcon500(1), //
      ShooterConstants.kGearRatio, // gear ratio
      ShooterConstants.kMomentOfInertia // moment of inertia kg m^2
  );

  private FlywheelSim upperWheelSim = new FlywheelSim(
      DCMotor.getFalcon500(1), //
      ShooterConstants.kGearRatio, // gear ratio
      ShooterConstants.kMomentOfInertia // moment of inertia kg m^2
  );

  public Shooter() {

    lower.config_kP(0, 0.08);
    //lower.config_kD(0, 5.00);
    lower.config_kF(0, 1023.0 / 21840.0);
    
    upper.config_kP(0, 0.08);
    //lower.config_kD(0, 5.00);
    upper.config_kF(0, 1023.0 / 21840.0);

    lower.setInverted(TalonFXInvertType.Clockwise);
    upper.setInverted(TalonFXInvertType.Clockwise);

    m_upperVelocitySetpoint = 0;
    m_lowerVelocitySetpoint = 0;
  }

  /** 
   * setLowerVoltage sets the voltage, in volts, of the lower flywheel.
   *
   * A positive voltage will move the wheel in the direction that launches
   * the ball out of the robot.
   */
  public void setLowerVoltage(double volts) {
    if (volts == 0.0) {
        m_lowerVelocitySetpoint = 0.0;
    }
    lower.set(TalonFXControlMode.PercentOutput, volts / General.kNominalBatteryVoltage);
  }

  /** 
   * setLowerVoltage sets the voltage, in volts, of the lower flywheel.
   *
   * A positive voltage will move the wheel in the direction that launches
   * the ball out of the robot.
   */
  public void setUpperVoltage(double volts) {
    if (volts == 0.0) {
        m_upperVelocitySetpoint = 0.0;
    }
    upper.set(TalonFXControlMode.PercentOutput, volts / General.kNominalBatteryVoltage);
  }

  /** 
   * getLowerVoltage returns the measured output voltage, in volts,
   * of the TalonFX controlling the lower flywheel.
   */
  public double getLowerVoltage() {
    return lower.getMotorOutputVoltage();
  }

  /** 
   * getUpperVoltage returns the measured output voltage, in volts,
   * of the TalonFX controlling the lower flywheel.
   */
  public double getUpperVoltage() {
    return upper.getMotorOutputVoltage();
  }

  /** 
   * setLowerVelocity sets the velocity, in RPM, of the lower flywheel.
   *
   * A positive velocity will move the wheel in the direction that launches
   * the ball out of the robot.
   */
  public void setLowerVelocity(double rpm) {
    SmartDashboard.putNumber("targetLowerVelocity(rpm)", rpm);
    m_lowerVelocitySetpoint = rpm;
    lower.set(TalonFXControlMode.Velocity, convertRPMToUnitsPer100MS(rpm));
  }

  /** 
   * setUpperVelocity sets the velocity, in RPM, of the upper flywheel.
   *
   * A positive velocity will move the wheel in the direction that launches
   * the ball out of the robot.
   */
  public void setUpperVelocity(double rpm) {
    SmartDashboard.putNumber("targetUpperVelocity(rpm)", rpm);
    m_upperVelocitySetpoint = rpm;
    upper.set(TalonFXControlMode.Velocity, convertRPMToUnitsPer100MS(rpm));
  }

  /** 
   * getLowerVoltage returns the measured velocity, in RPM, of the TalonFX
   * controlling the lower flywheel.
   */
  public double getLowerVelocity() {
    return convertUnitsPer100MSToRPM(lower.getSelectedSensorVelocity());
  }

  /** 
   * getUpperVoltage returns the measured velocity, in RPM, of the TalonFX
   * controlling the upper flywheel.
   */
  public double getUpperVelocity() {
    return convertUnitsPer100MSToRPM(upper.getSelectedSensorVelocity());
  }

  /** 
   * isAtDesiredVelocity returns true is both upper and lower are at the desired velocity, in RPM
   */
  public boolean isAtDesiredVelocity() {
    double deltaL = Math.abs(m_lowerVelocitySetpoint - getLowerVelocity());
    double deltaU = Math.abs(m_upperVelocitySetpoint - getUpperVelocity());

    if (deltaL <= (ShooterConstants.kVelocityTolerancePercent * m_lowerVelocitySetpoint) &&
        deltaU <= (ShooterConstants.kVelocityTolerancePercent * m_upperVelocitySetpoint)) {
            return true;
    }
    else return false;
  }


  private double convertUnitsPer100MSToRPM(double units) {
    return units * ShooterConstants.kUnitsPer100MSToRPMConversion;
  }

  private double convertRPMToUnitsPer100MS(double rpm) {
    return rpm / ShooterConstants.kUnitsPer100MSToRPMConversion;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("lowerVolts", getLowerVoltage());
    SmartDashboard.putNumber("upperVolts", getUpperVoltage());

    SmartDashboard.putNumber("lowerVelocity(rpm)", getLowerVelocity());
    SmartDashboard.putNumber("upperVelocity(rpm)", getUpperVelocity());

    SmartDashboard.putNumber("lowerShooterCurrent", lower.getStatorCurrent());
    SmartDashboard.putNumber("upperShooterCurrent", upper.getStatorCurrent());
  }

  @Override
  public void simulationPeriodic() {
    lowerWheelSim.setInputVoltage(getLowerVoltage());
    upperWheelSim.setInputVoltage(getUpperVoltage());

    lowerWheelSim.update(Robot.kDefaultPeriod);
    upperWheelSim.update(Robot.kDefaultPeriod);

    int lowerticksPer100ms = (int) convertRPMToUnitsPer100MS(lowerWheelSim.getAngularVelocityRPM());
    int upperticksPer100ms = (int) convertRPMToUnitsPer100MS(upperWheelSim.getAngularVelocityRPM());

    lower.getSimCollection().setIntegratedSensorVelocity(lowerticksPer100ms);
    upper.getSimCollection().setIntegratedSensorVelocity(upperticksPer100ms);

    lower.getSimCollection().setBusVoltage(RobotController.getBatteryVoltage());
    upper.getSimCollection().setBusVoltage(RobotController.getBatteryVoltage());
  }

  private static WPI_TalonFX createTalon(int deviceNumber) {
    var talon = new WPI_TalonFX(deviceNumber);

    /* Factory Default all hardware to prevent unexpected behaviour */
    CTREConfigs.configTalonFactoryDefault(talon, "Error setting factory default for shooter" + deviceNumber);

    CTREConfigs.setTalonStatusFramePeriod(talon, StatusFrameEnhanced.Status_1_General, Constants.General.kTalonCANStatusSlowRateInMs, "Error setting status_1 period for shooter " + deviceNumber);

    CTREConfigs.checkCtreError(talon.configVoltageCompSaturation(General.kNominalBatteryVoltage), "Error setting voltage compensation for shooter " + deviceNumber);
    talon.enableVoltageCompensation(true);
    talon.setNeutralMode(NeutralMode.Coast);

    /* Config neutral deadband to be the smallest possible */
    // TODO: ask Bart why he added this since we won't be outputting < 4%
    //talon.configNeutralDeadband(0.001);

    /* Config sensor used for Primary PID [Velocity] */
    CTREConfigs.checkCtreError(talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, General.kCANConfigTimeout), "Error setting shooter feedback sensor");

    /* Config the peak and nominal outputs */
    //talon.configNominalOutputForward(0);
    //talon.configNominalOutputReverse(0);
    //talon.configPeakOutputForward(1);
    //talon.configPeakOutputReverse(-1);

    return talon;
  }
}

