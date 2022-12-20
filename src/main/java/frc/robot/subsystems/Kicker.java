// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.Constants.KickerConstants;


public class Kicker extends SubsystemBase {
    
    // Creates a new Kicker Subsystem

    
    //need motor id and determine if a top sensor is used
    private WPI_TalonFX m_kicker = new WPI_TalonFX(KickerConstants.kKickerMotorID);
    

    public Kicker() {
        CTREConfigs.configTalonFactoryDefault(m_kicker, "Error configuring kicker motor factory defaults");
        CTREConfigs.setTalonStatusFramePeriod(m_kicker, StatusFrameEnhanced.Status_1_General, Constants.General.kTalonCANStatusSlowRateInMs, "Error setting kicker Talon status_1 period");
        CTREConfigs.setTalonStatusFramePeriod(m_kicker, StatusFrameEnhanced.Status_2_Feedback0, Constants.General.kTalonCANStatusSlowRateInMs, "Error setting kicker Talon status_2 period");
        m_kicker.setNeutralMode(NeutralMode.Brake);
        m_kicker.setInverted(TalonFXInvertType.CounterClockwise);
        // we want to enable very slight back pressure, so reduce default deadzone
        m_kicker.configNeutralDeadband(0.01);
    }

    public void rotate(double power) {
        m_kicker.set(ControlMode.PercentOutput, power);
    }
}