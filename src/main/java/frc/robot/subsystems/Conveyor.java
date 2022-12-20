// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.Constants.ConveyorConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Conveyor extends SubsystemBase {
    
    private WPI_TalonFX m_convMotor = new WPI_TalonFX(ConveyorConstants.kConveyorMotorID); 
    
     
    public Conveyor() {
        CTREConfigs.configTalonFactoryDefault(m_convMotor, "Error configuring conveyor motor factory defaults");
        CTREConfigs.setTalonStatusFramePeriod(m_convMotor, StatusFrameEnhanced.Status_1_General, Constants.General.kTalonCANStatusSlowRateInMs, "Error setting conveyor Talon status_1 period");
        CTREConfigs.setTalonStatusFramePeriod(m_convMotor, StatusFrameEnhanced.Status_2_Feedback0, Constants.General.kTalonCANStatusSlowRateInMs, "Error setting conveyor Talon status_2 period");
        m_convMotor.setInverted(TalonFXInvertType.Clockwise);
        m_convMotor.setNeutralMode(NeutralMode.Brake);
        // we want the option for very slight belt movement, so reduce default deadzone
        m_convMotor.configNeutralDeadband(0.01);
    }

    public void activate(double power) {

        m_convMotor.set(ControlMode.PercentOutput, power);
        SmartDashboard.putNumber("Conveyor stator current", m_convMotor.getStatorCurrent());
    }
}