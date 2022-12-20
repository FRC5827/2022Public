// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.General;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

public class IntakeArm extends SubsystemBase {

  public enum IntakeState {
    IntakeUp(IntakeConstants.kIntakeUpSensorDegrees), IntakeDown(IntakeConstants.kIntakeDownSensorDegrees);
    private double value;

    IntakeState(double value) {
      this.value = value;
    }

    public double getValue() {
      return this.value;
    }
  }

  // Motor controller used to move and change Intake's position
  private CANSparkMax m_armMotor = new CANSparkMax(IntakeConstants.kIntakeArmMotorID, MotorType.kBrushless);
  private CANCoder m_canCoder = new CANCoder(IntakeConstants.kIntakeCanCoderID);


  /** Creates a new Intake. */
  public IntakeArm(RobotContainer robotContainer) {
    
    m_armMotor.restoreFactoryDefaults();
    m_armMotor.setCANTimeout(General.kCANConfigTimeout);
    m_armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, Constants.General.kTalonCANStatusSlowRateInMs);
    m_armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, Constants.General.kTalonCANStatusSlowRateInMs);
    m_armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, Constants.General.kTalonCANStatusVerySlowRateInMs);
    m_armMotor.enableVoltageCompensation(Constants.General.kNominalBatteryVoltage);
    m_armMotor.setSmartCurrentLimit(Constants.IntakeConstants.kIntakeArmCurrentLimit);
    m_armMotor.setIdleMode(IdleMode.kCoast);
    m_armMotor.setInverted(false);

    CTREConfigs.configCANCoderFactoryDefault(m_canCoder, "Error configuring CANcoder m_intakeArm default settings");
    CANCoderConfiguration intakeCanConfig = new CANCoderConfiguration();
  
    intakeCanConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    intakeCanConfig.sensorDirection = true;
    intakeCanConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    intakeCanConfig.sensorTimeBase = SensorTimeBase.PerSecond;

    CTREConfigs.checkCtreError(m_canCoder.configAllSettings(intakeCanConfig, Constants.General.kCANConfigTimeout), "Error configuring CANCoder m_intakeArm");

  
  }

  public void armMotorSpeed(double speed) {
    // speed is controlled by command with WPI PID and external Talon encoder input
    m_armMotor.set(speed);
    SmartDashboard.putNumber("intake arm output", speed);
    SmartDashboard.putNumber("intake arm current", m_armMotor.getOutputCurrent());
  }

  public IntakeState getState() {
    double range = Math.abs(IntakeConstants.kIntakeUpSensorDegrees - IntakeConstants.kIntakeDownSensorDegrees);

    if (Math.abs(getIntakeEncoderPosition()) > range / 2) {
        return IntakeState.IntakeDown;
    }
    else return IntakeState.IntakeUp;
  }




  public double getIntakeEncoderPosition() {
    return m_canCoder.getPosition();
}


public void resetIntakeArmEncoder() {
  m_canCoder.setPosition(0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getState() == IntakeState.IntakeUp) {
        SmartDashboard.putBoolean("Intake Up", true);
    }
    else SmartDashboard.putBoolean("Intake Up", false);
    SmartDashboard.putNumber("intake encoder",  getIntakeEncoderPosition());
  }
    
}
