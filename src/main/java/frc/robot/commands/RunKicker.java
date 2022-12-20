// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Kicker;


public class RunKicker extends CommandBase{
    
 
    private final Kicker m_kicker;
    private double m_power;
    
    public RunKicker(Kicker kicker, double power) {
        // Declare Subsytem Dependencies
        addRequirements(kicker);

        // Subsytems
        m_kicker = kicker;
        m_power = power;
    }


    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_kicker.rotate(m_power);

    }

    // Called once when the command is ended or interrupted
    @Override
    public void end(boolean interrupted) {
        m_kicker.rotate(0.0);

    }

    // Returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;

    }
}
