// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterSetVelocity extends CommandBase {
  private final Shooter m_shooter;

  private double m_lowerVelocity, m_upperVelocity;
  

  /** Creates a new ShooterSetVelocity. */
  public ShooterSetVelocity(Shooter shooter, double lowerVelocity, double upperVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    m_shooter = shooter;

    m_lowerVelocity = lowerVelocity;
    m_upperVelocity = upperVelocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setLowerVelocity(m_lowerVelocity);
    m_shooter.setUpperVelocity(m_upperVelocity);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setLowerVoltage(0);
    m_shooter.setUpperVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
