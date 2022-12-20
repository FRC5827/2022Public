// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

//import org.photonvision.targeting.PhotonPipelineResult;
//import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.TurnPIDConstants;
import frc.robot.subsystems.LimelightSubsystem;
//import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveDrive;


public class DrivePointingToTarget extends CommandBase {

    private boolean m_fieldRelative;
    
    private SwerveDrive m_swerveDrive;
    private XboxController m_controller;
    //private PhotonVisionSubsystem m_photonVision;
    private LimelightSubsystem m_limelight;
    private int m_translationAxis;
    private int m_strafeAxis;
    private int m_rotationAxis;

    private SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(Constants.kJoystickSlewRate);
    private SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(Constants.kJoystickSlewRate);
    private SlewRateLimiter m_rSpeedLimiter = new SlewRateLimiter(Constants.kJoystickSlewRate);

    private PIDController m_turnPIDController;

    private NetworkTableEntry m_turnP_NTEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("TurnTargetP");
    private NetworkTableEntry m_turnI_NTEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("TurnTargetI");
    private NetworkTableEntry m_turnD_NTEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("TurnTargetD");
  

    // Driver control
    public DrivePointingToTarget(SwerveDrive swerveDrive, LimelightSubsystem limelightSubsystem, XboxController controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative) {
        m_swerveDrive = swerveDrive;
        //m_photonVision = photonVision;
        m_limelight = limelightSubsystem;
        addRequirements(swerveDrive);

        m_controller = controller;
        m_translationAxis = translationAxis;
        m_strafeAxis = strafeAxis;
        m_rotationAxis = rotationAxis;

        m_fieldRelative = fieldRelative;

        m_turnP_NTEntry.setDouble(TurnPIDConstants.kpTurnRio);
        m_turnI_NTEntry.setDouble(TurnPIDConstants.kiTurnRio);
        m_turnD_NTEntry.setDouble(TurnPIDConstants.kdTurnRio);
    }

    @Override
    public void initialize() {
        double p = m_turnP_NTEntry.getDouble(0.0);
        double i = m_turnI_NTEntry.getDouble(0.0);
        double d = m_turnD_NTEntry.getDouble(0.0);

        m_turnPIDController = new PIDController(p, i, d);

        m_turnPIDController.reset();
        m_turnPIDController.setTolerance(1.0);
        m_turnPIDController.disableContinuousInput();
        m_turnPIDController.setSetpoint(0.0);
    }

    @Override
    public void execute() {
        boolean hasTarget = m_limelight.hasValidTarget();
        double xOffsetDegs = m_limelight.getTX();

        double xVel = calculateXVel();
        double yVel = calculateYVel();
        double rVel = calculateAngularVel(hasTarget, xOffsetDegs);

        m_swerveDrive.drive(xVel, yVel, rVel, m_fieldRelative, true);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.drive(0.0, 0.0, 0.0, m_fieldRelative, true);
    }

    private double calculateXVel() {
        double xAxis = -m_controller.getRawAxis(m_translationAxis);
        xAxis = (Math.abs(xAxis) < Constants.kJoystickDeadband) ? 0 : xAxis;
        xAxis = xAxis * xAxis * xAxis;
        xAxis = m_xSpeedLimiter.calculate(xAxis);
        return xAxis * Constants.Swerve.maxSpeed;
    }

    private double calculateYVel() {
        double yAxis = -m_controller.getRawAxis(m_strafeAxis);
        yAxis = (Math.abs(yAxis) < Constants.kJoystickDeadband) ? 0 : yAxis;
        yAxis = yAxis * yAxis * yAxis;
        yAxis = m_ySpeedLimiter.calculate(yAxis);
        return yAxis * Constants.Swerve.maxSpeed;
    }

    private double calculateAngularVel(boolean hasTarget, double xOffsetDegs) {
        if (!hasTarget) {
            // No target found, use manual inputs.
            double rAxis = -m_controller.getRawAxis(m_rotationAxis);
            rAxis = Math.abs(rAxis) < Constants.kJoystickDeadband ? 0 : rAxis;
            rAxis = rAxis * rAxis * rAxis;
            rAxis = m_rSpeedLimiter.calculate(rAxis);
            return rAxis * Constants.Swerve.maxAngularVelocity;
        }

        // Target found, calulate angular velocity from offset.
        double rVel = m_turnPIDController.calculate(xOffsetDegs);
        return MathUtil.clamp(rVel, -0.12 * Constants.Swerve.maxAngularVelocity, 0.12 * Constants.Swerve.maxAngularVelocity);
    }
}
