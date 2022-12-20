// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeArm.IntakeState;

public class IntakeUpDown extends ProfiledPIDCommand {
  /** Creates a new MoveIntake. */

  private IntakeArm m_intakeArm;
  private double m_desiredPosition;

  public IntakeUpDown(IntakeArm intakeArm, IntakeState desiredState) {
    super(
      // set up PID Controller
      new ProfiledPIDController(
        IntakeConstants.kpIntakeRio,
        IntakeConstants.kiIntakeRio,
        IntakeConstants.kdIntakeRio,
        new TrapezoidProfile.Constraints(
          IntakeConstants.kIntakeArmMaxVelocityTicksPerS,
          IntakeConstants.kIntakeArmMaxAccelTicksPerSSquared)),
      // closed loop with ticks
      intakeArm::getIntakeEncoderPosition,
      // where we want to go (goal)
      desiredState.getValue(),
      (output, setpoint) -> intakeArm.armMotorSpeed(output),
      intakeArm);

    getController().setTolerance(IntakeConstants.kIntakeArmPositionTolerance);

    m_intakeArm = intakeArm;
    m_desiredPosition = desiredState.getValue();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void execute() {
    super.execute();
    SmartDashboard.putNumber("intake arm desired position", m_desiredPosition);
    SmartDashboard.putNumber("intake arm goal", getController().getGoal().position);

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_intakeArm.armMotorSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (RobotBase.isSimulation()) return true;
    
    if (super.isFinished()) return true;
    
    if (getController().atGoal()) {
      return true;
    }
    else return false;
  }
}
