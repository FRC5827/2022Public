// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {

    private boolean m_fieldRelative;
    private boolean m_openLoop;
    private NetworkTableEntry m_fieldRelativeStateShuffleboard;
    private NetworkTableEntry m_fieldRelativeStatusShuffleboard;
    
    private SwerveDrive m_swerveDrive;
    private XboxController m_controller;
    private int m_translationAxis;
    private int m_strafeAxis;
    private int m_rotationAxis;

    private SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(Constants.kJoystickSlewRate);
    private SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(Constants.kJoystickSlewRate);
    private SlewRateLimiter rSpeedLimiter = new SlewRateLimiter(Constants.kJoystickSlewRate);


    // Driver control
    public TeleopSwerve(SwerveDrive swerveDrive, XboxController controller, int translationAxis, int strafeAxis, int rotationAxis, boolean openLoop) {
        m_swerveDrive = swerveDrive;
        addRequirements(swerveDrive);

        m_controller = controller;
        m_translationAxis = translationAxis;
        m_strafeAxis = strafeAxis;
        m_rotationAxis = rotationAxis;
        m_fieldRelative = true;
        m_openLoop = openLoop;

        // button for toggle and status indicator
        m_fieldRelativeStateShuffleboard = NetworkTableInstance.getDefault().getTable("Shuffleboard").getEntry("Drivetrain/Field Relative");
        m_fieldRelativeStatusShuffleboard = NetworkTableInstance.getDefault().getTable("Shuffleboard").getEntry("Drivetrain/FieldRelativeStatus");
    }

    @Override
    public void execute() {
        // xbox controller provides negative values when pushing forward (y axis), so negate
        // negate strafe (left/right x axis stick) as we want positive when pushing left (positive y on field)
        // negate rotation as we want positive value when when pushing left (CCW is postive)
        double yAxis = -m_controller.getRawAxis(m_translationAxis);
        double xAxis = -m_controller.getRawAxis(m_strafeAxis);
        double rAxis = -m_controller.getRawAxis(m_rotationAxis);

        if (m_fieldRelativeStateShuffleboard != null) {
            boolean dashboardSwitch = m_fieldRelativeStateShuffleboard.getBoolean(true);
            if (dashboardSwitch) {
                m_fieldRelative = true;
            }
            else
            {
                m_fieldRelative = false;
            }
        }

        // update current field relative status on Shuffleboard
        if (m_fieldRelativeStatusShuffleboard != null) {
            m_fieldRelativeStatusShuffleboard.setBoolean(m_fieldRelative);
        }

        // apply deadband
        yAxis = (Math.abs(yAxis) < Constants.kJoystickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.kJoystickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.kJoystickDeadband) ? 0 : rAxis;

        // curve inputs
        yAxis = Math.abs(yAxis) * Math.abs(yAxis) * yAxis;
        xAxis = Math.abs(xAxis) * Math.abs(xAxis) * xAxis;
        rAxis = Math.abs(rAxis) * Math.abs(rAxis) * rAxis;

        // uncomment to scale down output
        //yAxis *= 0.5;
        //xAxis *= 0.5;
        //rAxis *= 0.5;

        // slew rate limiter
        yAxis = ySpeedLimiter.calculate(yAxis);
        xAxis = xSpeedLimiter.calculate(xAxis);
        rAxis = rSpeedLimiter.calculate(rAxis);
        
        // controller yAxis is forward movement, so is passed as x component of the translation
        // multiply [-1, 1] user input by velocities as these are desired chassis speeds
        double forward = yAxis * Constants.Swerve.maxSpeed;
        double strafe = xAxis * Constants.Swerve.maxSpeed;
        double rotation = rAxis * Constants.Swerve.maxAngularVelocity;

        m_swerveDrive.drive(forward, strafe, rotation, m_fieldRelative, m_openLoop);
    }
}
