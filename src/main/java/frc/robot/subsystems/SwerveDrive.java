// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Robot;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.SwerveModule;


public class SwerveDrive extends SubsystemBase {
    public SwerveDrivePoseEstimator m_swervePoseEstimator;
    //public SwerveDriveOdometry m_swerveOdometry;
    public SwerveModule[] m_swerveModules;
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    private RobotContainer m_robotContainer;

    // The Field2d class shows the field in the sim GUI
    private Field2d m_fieldSim;
    private double m_yawSim = 0.0;

    private double[] m_moduleDistanceLast;
    private double m_tickLast;
    private double m_dt;
    private Timer m_timer;

    // offset between actual yaw from gyro and what we think is "zero"
    private double m_yawOffset = 0.0;

    private int m_periodicCounter = 0;

    private final ShuffleboardTab m_driveShuffleboardTab = Shuffleboard.getTab("Drivetrain");
    private final NetworkTableEntry m_xEntry = m_driveShuffleboardTab.add("X", 0.0)
                                                    .withPosition(0, 1)
                                                    .withSize(1, 1)
                                                    .getEntry();
    private final NetworkTableEntry m_yEntry = m_driveShuffleboardTab.add("Y", 0.0)
                                                    .withPosition(0, 2)
                                                    .withSize(1, 1)
                                                    .getEntry();
    private final NetworkTableEntry m_Heading = m_driveShuffleboardTab.add("Heading", 0.0)
                                                    .withPosition(0, 3)
                                                    .withSize(1, 1)
                                                    .getEntry();
    private final NetworkTableEntry m_Pose = m_driveShuffleboardTab.add("Pose", "No Data")
                                                    .withPosition(2, 0)
                                                    .withSize(4, 1)
                                                    .getEntry();


    public SwerveDrive(RobotContainer robotContainer) {
        m_robotContainer = robotContainer;

        Pose2d initialPose = new Pose2d();
        
        //m_swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.kSwerveKinematics, getR2dHeading(), initialPose);

        m_swervePoseEstimator = new SwerveDrivePoseEstimator(getR2dHeading(), initialPose, Constants.Swerve.kSwerveKinematics, 
                                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.01, 0.01, 0.01),    // model state std deviations (x, y, theta)
                                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.01),                // local measurement (encoder/gyro) std deviations
                                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1));      // vision measurement std deviation (x, y, theta)

        m_swerveModules = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Module0.constants),
            new SwerveModule(1, Constants.Swerve.Module1.constants),
            new SwerveModule(2, Constants.Swerve.Module2.constants),
            new SwerveModule(3, Constants.Swerve.Module3.constants)
        };

        SmartDashboard.putData("/SwerveDrive/SwerveModules/FrontLeft", m_swerveModules[0]);
        SmartDashboard.putData("/SwerveDrive/SwerveModules/FrontRight", m_swerveModules[1]);
        SmartDashboard.putData("/SwerveDrive/SwerveModules/BackLeft", m_swerveModules[2]);
        SmartDashboard.putData("/SwerveDrive/SwerveModules/BackRight", m_swerveModules[3]);

        m_timer = new Timer();
        m_timer.reset();
        m_timer.start();
        m_tickLast = 0;

        zeroHeading();

        m_moduleDistanceLast = new double[m_swerveModules.length];
        for (SwerveModule module : m_swerveModules) {
            m_moduleDistanceLast[module.m_number] = module.getDriveEncoderDistanceMeters();
        }

        m_fieldSim = new Field2d();
        SmartDashboard.putData("Field", m_fieldSim);

        if (RobotBase.isSimulation()) {
            // add sim stuff here as needed
        }
    }


    public void drive(double forward, double strafe, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SmartDashboard.putNumber("/SwerveDrive/forward", forward);
        SmartDashboard.putNumber("/SwerveDrive/strafe", strafe);
        SmartDashboard.putNumber("/SwerveDrive/rotation", rotation);
        SmartDashboard.putBoolean("/SwerveDrive/fieldRelative", fieldRelative);
        SmartDashboard.putBoolean("/SwerveDrive/isOpenLoop", isOpenLoop);

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.kSwerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    forward, 
                                    strafe, 
                                    rotation, 
                                    getR2dHeading()
                                )
                                : new ChassisSpeeds(
                                    forward, 
                                    strafe, 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule module : m_swerveModules) {
            module.setDesiredState(swerveModuleStates[module.m_number], isOpenLoop);
        }
    }    

    // used by SwerveControllerCommand or PPSwerveControllerCommand during autonomous
    // these states are all executed in closed loop mode with on-board motor PID for velocity
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for (SwerveModule module : m_swerveModules) {
            module.setDesiredState(desiredStates[module.m_number], false);
        }
    }    

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[m_swerveModules.length];
        for (SwerveModule module : m_swerveModules) {
            states[module.m_number] = module.getState();
        }
        return states;
    }

    public Pose2d getPose() {
        return m_swervePoseEstimator.getEstimatedPosition();
        //return m_swerveOdometry.getPoseMeters();
    }

    public void resetOdometryAndAngleOffset(Pose2d oldPose, boolean setHeadingToZero) {
        for (SwerveModule module : m_swerveModules) {
            module.resetDriveEncoder();
        
            try {
                // I hate using delays, however setting drive encoder to 0 doesn't seem to take effect right away (at least in sim)
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        m_timer.reset();
        m_timer.start();
        m_tickLast = 0;

        m_moduleDistanceLast = new double[m_swerveModules.length];
        for (SwerveModule module : m_swerveModules) {
            m_moduleDistanceLast[module.m_number] = module.getDriveEncoderDistanceMeters();
            //System.out.println("distance: " + module.getDriveEncoderDistanceMeters());
        }

        System.out.println("reset zeroHeading: " + setHeadingToZero + ", oldPose: " + oldPose);
        
        if (setHeadingToZero == true) {
            if (Robot.isReal()) {
                m_gyro.zeroYaw();
                m_yawOffset = m_gyro.getYaw();
            }
            else {
                m_yawSim = 0.0;
                m_yawOffset = m_yawSim;
            }
            System.out.println("reset yaw offset: " + m_yawOffset);
            System.out.println("reset gyro heading: " + getHeading());
        }
        else {
            m_yawOffset = oldPose.getRotation().getDegrees();
            //System.out.println("reset offset: " + oldPose.getRotation().getDegrees());
        }

        Rotation2d newHeading = getR2dHeading();

        //System.out.println("reset newHeading: " + newHeading);

        //m_swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.kSwerveKinematics, newHeading, oldPose);

        m_swervePoseEstimator = new SwerveDrivePoseEstimator(newHeading, oldPose, Constants.Swerve.kSwerveKinematics, 
                                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.01, 0.01, 0.01),    // model state std deviations (x, y, theta)
                                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.01),                // local measurement (encoder/gyro) std deviations
                                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1));      // vision measurement std deviation (x, y, theta)

    }


    /**
     * Zeroes the heading of the robot.
     * This method also resets the pose so that it doesn't appear
     * that the robot has changed positon.
     */
    public void zeroHeading() {
        Pose2d currentPose = getPose();

        //System.out.println("zeroHeading");

        resetOdometryAndAngleOffset(currentPose, true);
    }

    public Rotation2d getR2dHeading() {
        return (Rotation2d.fromDegrees(getHeading()));
    }

    /**
     * Used to set brake or neutral mode for motors on swerve module
     * 
     * @param mode if true, enable brake mode
     */
    public void setBrakeMode(boolean mode) {
        for (SwerveModule module : m_swerveModules) {
            module.setBrakeMode(mode);
        }
    }

    /**
     * Moves swerve modules to zero angle position
     */
    public void zeroModuleAngles() {
        for (SwerveModule module : m_swerveModules) {
            module.zeroModuleAngle();
        }
    }

    /**
     * Returns the heading of the robot from the IMU.
     *
     * @return the robot's heading in degrees, from -360 to 360
     */
    public double getHeading() {
        double heading;

        if (Robot.isReal()) {
            heading = m_gyro.getYaw() * (Constants.Swerve.invertGyro ? -1.0 : 1.0);
        }
        else {
            heading = m_yawSim * (Constants.Swerve.invertGyro ? -1.0 : 1.0);
        }
        heading = Math.IEEEremainder(heading + m_yawOffset, 360);
        return heading;

        //return (Math.IEEEremainder(m_gyro.getAngle(), 360) * (Constants.Swerve.invertGyro ? -1.0 : 1.0));
    }

    @Override
    public void periodic() {
        double tick = m_timer.get();
        m_dt = tick - m_tickLast;
        m_tickLast = tick;

        double[] moduleDistances = new double[m_swerveModules.length];
        SwerveModuleState[] moduleStates = new SwerveModuleState[m_swerveModules.length];

        try {
            for (SwerveModule module : m_swerveModules) {
                moduleDistances[module.m_number] = module.getDriveEncoderDistanceMeters();
                moduleStates[module.m_number] = new SwerveModuleState((moduleDistances[module.m_number] - m_moduleDistanceLast[module.m_number]) / m_dt, Rotation2d.fromDegrees(module.getAngle()));
            }
            // update pose estimator/odometry
            //m_swerveOdometry.updateWithTime(tick, getR2dHeading(), moduleStates);
            m_swervePoseEstimator.updateWithTime(tick, getR2dHeading(), moduleStates);
            m_moduleDistanceLast = moduleDistances;
        }
        catch (ArithmeticException e) {
            System.out.println("Division by zero in SwerveDrive periodic updating odometry");
        }
        
        // using tick with updated encoder position seems more accurate than using Talon's velocity measurement
        //m_swerveOdometry.update(getR2dHeading(), getStates());
        //m_swervePoseEstimator.update(getR2dHeading(), getStates());
        
        //ChassisSpeeds chassisSpeeds = Constants.Swerve.kSwerveKinematics.toChassisSpeeds(getStates());
        //SmartDashboard.putNumber("currentV", chassisSpeeds.vxMetersPerSecond);

        if (m_robotContainer.hasVisionTarget()) {
            // use with 2023 WPILib (new methods)
            double distanceToTarget = m_robotContainer.getVisionDistanceToTarget();
            double targetYaw = m_robotContainer.getVisionYawToTarget();
            //Transform2d camToTarget = estimateCameraToTarget(new Translation2d(distanceToTarget, targetYaw), Constants.LimelightVals.targetPose2dLocation, getR2dHeading());
        
            // robot relative to camera, so 10 inches in front and 4 inches to the right of the camera in field coordinates
            //Pose2d estimatedPose = estimateFieldToRobot(camToTarget, Constants.LimelightVals.targetPose2dLocation, new Transform2d(new Translation2d(10.0, -4.0), new Rotation2d()));
            //m_swervePoseEstimator.addVisionMeasurement(estimatedPose, tick);
        }
    
        // get updated position
        Pose2d currentPose = getPose();

        m_fieldSim.setRobotPose(currentPose);
        m_Heading.setDouble(currentPose.getRotation().getDegrees());

        // pose string creation is relatively expensive, so only update stats occasionally
        if (m_periodicCounter % 15 == 0) {
            m_xEntry.setDouble(currentPose.getX());
            m_yEntry.setDouble(currentPose.getY());

            m_Pose.setString(currentPose.toString());
        }
 
        m_periodicCounter++;
    }

    @Override
    public void simulationPeriodic() {

        for (SwerveModule module : m_swerveModules) {
            module.simulationPeriodic(m_dt);
        }

        ChassisSpeeds chassisSpeeds = Constants.Swerve.kSwerveKinematics.toChassisSpeeds(getStates());
        double angularVelocityDegrees = Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond);
       
        // update simulated NavX
        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        double degrees = (m_yawSim - (angularVelocityDegrees * m_dt));
        angle.set(degrees);
        m_yawSim = degrees;

        //if (m_periodicCounter % 40 == 0) {
        //    System.out.println("yaw: " + m_gyro.getYaw() + ", deg: " + degrees + ", offset: " + m_yawOffset + ", getHeading: " + getHeading());
        //    System.out.println(("Pose: " + m_swervePoseEstimator.getEstimatedPosition()));
        //}

    }

    public double getChassisSpeed() {
        ChassisSpeeds chassisSpeeds = Constants.Swerve.kSwerveKinematics.toChassisSpeeds(getStates());
        return chassisSpeeds.vxMetersPerSecond;
        // returns Chassis Velcoity in the x direction
    }

}