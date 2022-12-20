// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.TreeMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;


public class ShootWithVision extends CommandBase {


  private static class ShooterRPM {
    public double m_upperRPM;
    public double m_lowerRPM;

    public ShooterRPM(Double upperRPM, Double lowerRPM) {
      m_upperRPM = upperRPM;
      m_lowerRPM = lowerRPM;
    }

    public static ShooterRPM lerp(ShooterRPM near, ShooterRPM far, double percent) {
      percent = MathUtil.clamp(percent, 0.0, 1.0);

      double upperRPM = (1.0 - percent) * near.m_upperRPM + percent * far.m_upperRPM;
      double lowerRPM = (1.0 - percent) * near.m_lowerRPM + percent * far.m_lowerRPM;

      return new ShooterRPM(upperRPM, lowerRPM);
    }
  }

  private final LimelightSubsystem m_limelightSubsystem;
  private final Shooter m_shooter;
  private Timer m_timer;
  private double m_upperRPMToSet;
  private double m_lowerRPMToSet;

  // replace with 2023 WPILib's InterpolatingTreeMap<K, V> when available
  private final TreeMap<Double, ShooterRPM> m_shooterTable = new TreeMap<Double, ShooterRPM>();

  /** Creates a new ShootWithVision. 
   *  Assumes that we are already in line with target such that tx (target offset) is close to 0.
  */
  public ShootWithVision(LimelightSubsystem limelightSubsystem, Shooter shooter, boolean bLower) {
    m_limelightSubsystem = limelightSubsystem;
    m_shooter = shooter;
    m_timer = new Timer();

    SmartDashboard.putNumber("shooterDistanceOffset", 0.0);

    if (bLower) {
        m_shooterTable.put(50.0, new ShooterRPM(1200.0, 750.0));
        m_shooterTable.put(196.0, new ShooterRPM(1200.0, 750.0));
    }
    else {
        // 2200, 1700 rpm at 50 -- LL said 50
        m_shooterTable.put(50.0, new ShooterRPM(2200.0, 1700.0));
        // 2250, 1700 rpm at 71 -- LL said 73
        m_shooterTable.put(73.0, new ShooterRPM(2250.0, 1700.0));
        // 2200, 2050 rpm at 100 -- LL said 101
        m_shooterTable.put(101.0, new ShooterRPM(2200.0, 2050.0));
        // 2200 rpm at 120 -- LL said 121
        m_shooterTable.put(121.0, new ShooterRPM(2200.0, 2200.0));
        // 2350 rpm at 143 -- LL said 140
        m_shooterTable.put(140.0, new ShooterRPM(2450.0, 2450.0));
        // from practice field
        m_shooterTable.put(159.0, new ShooterRPM(2700.0, 2700.0));
        // from testing on our own target 3/11/22
        //m_shooterTable.put(172.0, new ShooterRPM(2750.0, 2750.0));
        // 2850 rpm at 203 inches actual -- LL said 185
        m_shooterTable.put(185.0, new ShooterRPM(2950.0, 2950.0));
        m_shooterTable.put(199.0, new ShooterRPM(3100.0, 3100.0));

    }

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  
  @Override
  public void initialize() {

    m_timer.reset();
    m_timer.start();

    if (m_limelightSubsystem.hasValidTarget() == false) {
      // no target visible, so assume we are close to target
      m_upperRPMToSet = m_shooterTable.get(m_shooterTable.firstKey()).m_upperRPM;
      m_lowerRPMToSet = m_shooterTable.get(m_shooterTable.firstKey()).m_lowerRPM;
      return;
    }

    // we have a target -- get the distance
    double distance = m_limelightSubsystem.getDistance();


    double distanceWithYawComp = m_limelightSubsystem.getDistanceWithYaw();
    distanceWithYawComp += SmartDashboard.getNumber("shooterDistanceOffset", 0.0);

    // min distance in TreeMap
    double minDist = m_shooterTable.firstKey();

    // max distance in TreeMap
    double maxDist = m_shooterTable.lastKey();

    // clamp to min/max values in table so we are not out of bounds
    distance = MathUtil.clamp(distanceWithYawComp, minDist, maxDist);

    // get upper and lower keys around our distance
    double near = m_shooterTable.floorKey(distance);
    double far = m_shooterTable.ceilingKey(distance);

    // range of table entries around our distance
    double boundedDelta = far - near;
    double actualDelta = distance - near;
    double percent;
  
    // calculate percent based on range
    if (boundedDelta != 0.0) {
      percent = actualDelta / boundedDelta;
    }
    else {
      percent = 0.0;
    }

    // interpolate values
    m_upperRPMToSet = ShooterRPM.lerp(m_shooterTable.get(near), m_shooterTable.get(far), percent).m_upperRPM;
    m_lowerRPMToSet = ShooterRPM.lerp(m_shooterTable.get(near), m_shooterTable.get(far), percent).m_lowerRPM;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // override from network tables if set
    m_upperRPMToSet = SmartDashboard.getNumber("Shooter Upper RPM", m_upperRPMToSet);
    m_lowerRPMToSet = SmartDashboard.getNumber("Shooter Lower RPM", m_lowerRPMToSet);
    // set velocities
    m_shooter.setUpperVelocity(m_upperRPMToSet);
    m_shooter.setLowerVelocity(m_lowerRPMToSet);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setUpperVoltage(0.0);
    m_shooter.setLowerVoltage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.hasElapsed(2.0)) {
        return true;
    }
    else return false;
  }
}
