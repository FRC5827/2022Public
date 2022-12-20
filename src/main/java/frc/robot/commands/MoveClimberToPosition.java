// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class MoveClimberToPosition extends CommandBase {
  private Climber m_climber;
  private double m_position;
  
  /** Creates a new MoveClimberToPosition. */
  public MoveClimberToPosition(Climber climber, double position) {

    m_position = position;
    m_climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_climber.moveToPosition(m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_climber.climberPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      double position = m_climber.getCurrentPosition();

      if ((position >= (m_position - (ClimberConstants.kEncoderTicksPerRev * 2) )) &&
          (position <= (m_position + (ClimberConstants.kEncoderTicksPerRev * 2)))) {
          return true;
      }
      return false;
  }
}
