// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import static frc.robot.Constants.TurnPIDConstants;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * An example command that uses an example subsystem.
 */
public class TurnToTarget extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_driveSubsystem;
  private final LimelightSubsystem m_limelightSubsystem;
  private PIDController m_turnPIDController;
  private int m_counter;
  private NetworkTableEntry m_turnP_NTEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("TurnTargetP");
  private NetworkTableEntry m_turnI_NTEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("TurnTargetI");
  private NetworkTableEntry m_turnD_NTEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("TurnTargetD");

  /**
   * Creates a new robotTurnDegrees.
   *
   * @param driveSubsystem The DriveSubsystem used by this command.
   * @param limelightSubsystem The vision subsystem used by this command.
   */
  public TurnToTarget(SwerveDrive driveSubsystem, LimelightSubsystem limelightSubsystem) {

    m_driveSubsystem = driveSubsystem;
    m_limelightSubsystem = limelightSubsystem;
    m_counter = 0;

    m_turnP_NTEntry.setDouble(TurnPIDConstants.kpTurnRio);
    m_turnI_NTEntry.setDouble(TurnPIDConstants.kiTurnRio);
    m_turnD_NTEntry.setDouble(TurnPIDConstants.kdTurnRio);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double p = m_turnP_NTEntry.getDouble(0.0);
    double i = m_turnI_NTEntry.getDouble(0.0);
    double d = m_turnD_NTEntry.getDouble(0.0);

    System.out.println("TurnP: " + p);
    //System.out.println("TurnI: " + i);
    //System.out.println("TurnD: " + d);

    m_turnPIDController = new PIDController(p, i, d);
    m_turnPIDController.setTolerance(1.0);

    m_turnPIDController.reset();
    m_turnPIDController.disableContinuousInput();
    m_turnPIDController.setSetpoint(0.0);

    m_counter = 0;
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_limelightSubsystem.hasValidTarget() == true) {
      double rotation = m_turnPIDController.calculate(m_limelightSubsystem.getTX());
      rotation = MathUtil.clamp(rotation, -0.25 * Constants.Swerve.maxAngularVelocity, 0.25  * Constants.Swerve.maxAngularVelocity);
      SmartDashboard.putNumber("TTT rotation", rotation / Constants.Swerve.maxAngularVelocity);
      m_driveSubsystem.drive(0, 0, rotation, false, true);
    }
    else {
      m_turnPIDController.calculate(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // leave LED on as we probably will use in subsequent command or change pipeline
    m_driveSubsystem.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (RobotBase.isSimulation()) return true;

    if (m_limelightSubsystem.hasValidTarget() == false) {
      m_counter++;
    }
    else {
      m_counter = 0;
    }

    // if we lost target for 12 consecutive scheduled executes, then stop
    if (m_counter >= 12) {
      return true;
    }

    // Has PID loop reached it's setpoint?  If so, we're done.
    return (m_turnPIDController.atSetpoint());
  }
}
