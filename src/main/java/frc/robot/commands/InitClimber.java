// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.General;
import frc.robot.subsystems.Climber;

public class InitClimber extends CommandBase {
  private Climber m_climber;

  /** Creates a new InitClimber. */
  public InitClimber(Climber climber) {

    m_climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_climber.setStatusFramePeriod(General.kTalonCANStatusFastRateInMs);
      m_climber.setOpenLoopRamp(0.3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_climber.climberPower(-0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_climber.climberPower(0.0);
      m_climber.setOpenLoopRamp(0.0);
      m_climber.resetEncoder();
      m_climber.setStatusFramePeriod(General.kTalonCANStatusSlowRateInMs);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_climber.getStatorCurrent() > ClimberConstants.kClimberZeroStatorCurrentThreshhold) {
        return true;
    }
    return false;
  }
}
