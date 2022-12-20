// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.Constants.General;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class IntakeSpinner extends SubsystemBase {

  // Motor controllers used to move and change Intake's position
  private WPI_TalonFX m_intakeSpinMotor = new WPI_TalonFX(IntakeConstants.kIntakeSpinnerMotorID);

  /** Creates a new IntakeSpinner. */
  public IntakeSpinner() {

    CTREConfigs.configTalonFactoryDefault(m_intakeSpinMotor, "Error configuring intake spinner motor default settings");
    CTREConfigs.setTalonStatusFramePeriod(m_intakeSpinMotor, StatusFrameEnhanced.Status_1_General, Constants.General.kTalonCANStatusSlowRateInMs, "Error setting intake spinner Talon status_1 periodConstants.General.kTalonCANStatusSlowRateInMs");
    CTREConfigs.setTalonStatusFramePeriod(m_intakeSpinMotor, StatusFrameEnhanced.Status_2_Feedback0, Constants.General.kTalonCANStatusSlowRateInMs,"Error setting intake spinner Talon status_2 period");
    CTREConfigs.setTalonControlFramePeriod(m_intakeSpinMotor, ControlFrame.Control_3_General, General.kTalonCANControlPrimaryRateInMs, "Error setting intake spinner Talon control_3 period");
    m_intakeSpinMotor.setNeutralMode(NeutralMode.Coast);
    m_intakeSpinMotor.setInverted(TalonFXInvertType.CounterClockwise);
  }

  public void spin(double speed) {
      m_intakeSpinMotor.set(TalonFXControlMode.PercentOutput, speed);
      SmartDashboard.putNumber("intake spin output", speed);
  }


  @Override
  public void periodic() {
  }
}
