// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Conveyor;


public class RunConveyor extends CommandBase{
    
 
    private final Conveyor m_conveyor;
    private double m_power;
    
    public RunConveyor(Conveyor conveyor, double power) {
        // Declare Subsytem Dependencies
        addRequirements(conveyor);

        // Subsytems
        m_conveyor = conveyor;
        m_power = power;
    }


    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_conveyor.activate(m_power);

    }

    // Called once when the command is ended or interrupted
    @Override
    public void end(boolean interrupted) {
        m_conveyor.activate(0.0);

    }

    // Returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;

    }
}
