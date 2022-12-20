// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.KickerConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.IntakeSpinner;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.SwerveDrive;

public class IntakeSpin extends CommandBase {

  private IntakeSpinner m_intakeSpinner;
  private Conveyor m_conveyor;
  private Kicker m_kicker;
  private boolean m_bIsForward;
  private NetworkTableEntry m_intakeSpeedEntry = NetworkTableInstance.getDefault().getEntry("intakeSpeed");


  /** Creates a new IntakeSpin. */
  public IntakeSpin(IntakeSpinner intake, Conveyor conveyor, Kicker kicker, SwerveDrive swerveDrive, boolean bIsForward) {
    m_intakeSpinner = intake;
    m_conveyor = conveyor;
    m_kicker = kicker;
    m_bIsForward = bIsForward;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSpinner);
    addRequirements(m_conveyor);
    addRequirements(m_kicker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSpeedEntry.setNumber(IntakeConstants.kIntakeSpinForwardSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_bIsForward == true) {

      m_intakeSpinner.spin(m_intakeSpeedEntry.getDouble(0));
      m_conveyor.activate(ConveyorConstants.kIntakeSpeed);
      

      // if veolicty in the x direction is more than 3.9, the poweroutput would be 7- percent
      /* if (3.9 <= m_swerveDrive.getChassisSpeed() ) {
        m_intakeSubSystem.spin(0.7);
        m_conveyor.activate(ConveyorConstants.kIntakeSpeed);

      } else {
        m_intakeSubSystem.spin(IntakeConstants.kIntakeSpinForwardSpeed);
        m_conveyor.activate(ConveyorConstants.kIntakeSpeed);
      } */

    }
    else {
            m_intakeSpinner.spin(IntakeConstants.kIntakeSpinBackwardSpeed);
            m_conveyor.activate(ConveyorConstants.kBackwardFastSpeed);
            m_kicker.rotate(KickerConstants.kBackwardSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSpinner.spin(0.0);
    m_conveyor.activate(0.0);
    m_kicker.rotate(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
