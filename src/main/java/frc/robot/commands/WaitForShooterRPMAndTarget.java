// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;

public class WaitForShooterRPMAndTarget extends CommandBase {
    private final Shooter m_shooter;
    private final LimelightSubsystem m_limelight;

    /** Creates a new WaitForShooterRPM. */
    public WaitForShooterRPMAndTarget(Shooter shooter, LimelightSubsystem limelight) {
        m_shooter = shooter;
        m_limelight = limelight;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_shooter.isAtDesiredVelocity()) {
            if (m_limelight.hasValidTarget() && m_limelight.getTX() <= 2.0) {
                return true;
            }
            else if (!m_limelight.hasValidTarget()) {
                return true;
            }
        }
        return false;
    }
}
