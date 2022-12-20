// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.General;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Climber extends SubsystemBase {
    
    ElevatorFeedforward m_feedForward = new ElevatorFeedforward(ClimberConstants.kClimbKS, ClimberConstants.kClimbKG, ClimberConstants.kClimbKV);

    // Climber hook motor
    private WPI_TalonFX m_winchMotor = new WPI_TalonFX(ClimberConstants.kFalconID);

    public Climber() {

        StatorCurrentLimitConfiguration statorConfig = new StatorCurrentLimitConfiguration();

        statorConfig.currentLimit = ClimberConstants.kStatorCurrentLimit;
        statorConfig.triggerThresholdCurrent = ClimberConstants.kStatorCurrentThreshhold;
        statorConfig.triggerThresholdTime = ClimberConstants.kStatorCurrentThreshholdTime;
        statorConfig.enable = ClimberConstants.kStatorCurrentEnabled;

        // Resets to Factory Default and sets motor controller configuration
        CTREConfigs.configTalonFactoryDefault(m_winchMotor, "Error configuring climb motor default settings");
        // reduce CAN usage since we don't need general status data as frequently
        // Falcon on winch does not need quick response so increase period between control frames on CAN bus
        CTREConfigs.setTalonStatusFramePeriod(m_winchMotor, StatusFrameEnhanced.Status_1_General, General.kTalonCANStatusSlowRateInMs, "Error setting winch Talon Status_1 period");
        CTREConfigs.setTalonControlFramePeriod(m_winchMotor, ControlFrame.Control_3_General, General.kTalonCANControlPrimaryRateInMs, "Error setting winch primary Talon Control_3 period");

        m_winchMotor.setNeutralMode(NeutralMode.Brake);
        CTREConfigs.checkCtreError(m_winchMotor.configVoltageCompSaturation(General.kNominalBatteryVoltage, General.kCANConfigTimeout), "Error configuring climb motor voltage comp saturation"); 
        CTREConfigs.checkCtreError(m_winchMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, General.kCANConfigTimeout), "Error configuring climb motor encoder");
        CTREConfigs.checkCtreError(m_winchMotor.configFeedbackNotContinuous(true, General.kCANConfigTimeout), "Error setting climb motor continuous feedback mode");
        m_winchMotor.setSensorPhase(false);
        CTREConfigs.checkCtreError(m_winchMotor.configClosedLoopPeakOutput(0, ClimberConstants.kClimberClosedLoopPeakOutput, General.kCANConfigTimeout), "Error setting climb motor closed loop peak output");
        CTREConfigs.checkCtreError(m_winchMotor.config_kP(0, ClimberConstants.kClimbKP), "Error setting climb motor KP");
        CTREConfigs.checkCtreError(m_winchMotor.configStatorCurrentLimit(statorConfig, General.kCANConfigTimeout), "Error setting supply current for climb motor");
        CTREConfigs.checkCtreError(m_winchMotor.setSelectedSensorPosition(0, 0, General.kCANConfigTimeout), "Error setting winch motor sensor position");

        m_winchMotor.setInverted(TalonFXInvertType.Clockwise);
      
    }

    public void extend() {
        m_winchMotor.set(ControlMode.Position, ClimberConstants.kClimbPositionTicksTop);
    }

    public void retract() {
        // voltage necessary to prevent robot from falling (0 velocity)
        double feedForward = m_feedForward.calculate(0.0);
        m_winchMotor.set(ControlMode.Position, ClimberConstants.kClimbPositionTicksHome, DemandType.ArbitraryFeedForward, feedForward);
    }

    public void bottom() {
        m_winchMotor.set(ControlMode.Position, ClimberConstants.kClimbPositionTicksBottom);
    }

    public void home() {
        m_winchMotor.set(ControlMode.Position, ClimberConstants.kClimbPositionTicksHome);
    }

    public void moveToPosition(double position) {
        m_winchMotor.set(ControlMode.Position, position);
    }

    public void climberPower(double power) {
        m_winchMotor.set(ControlMode.PercentOutput, power);
    }

    public double getCurrentPosition() {
        return m_winchMotor.getSelectedSensorPosition();
    }

    public void setOpenLoopRamp(double seconds) {
        CTREConfigs.checkCtreError(m_winchMotor.configOpenloopRamp(seconds, General.kCANConfigTimeout), "Error setting winch motor open loop ramp time");
    }

    public void resetEncoder() {
        CTREConfigs.checkCtreError(m_winchMotor.setSelectedSensorPosition(0, 0, General.kCANConfigTimeout), "Error setting winch motor sensor position");
    }

    public double getStatorCurrent() {
        return m_winchMotor.getStatorCurrent();
    }

    public void setStatusFramePeriod(int period) {
        CTREConfigs.setTalonStatusFramePeriod(m_winchMotor, StatusFrameEnhanced.Status_1_General, period, "Error setting winch Talon Status_1 period from command");
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb encoder", m_winchMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Climb stator current", m_winchMotor.getStatorCurrent());
    }

}



