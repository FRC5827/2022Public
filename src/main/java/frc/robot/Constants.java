// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.SwerveModuleConstants;


public final class Constants {
    public static final double kJoystickDeadband = 0.08;
    public static final double kJoystickSlewRate = 4.0;

    public static final class General {
        public static final double kNominalBatteryVoltage = 12.0;
        public static final int kCANConfigTimeout = 400;
        public static final int kTalonCANStatusFastRateInMs = 10;
        public static final int kTalonCANStatusSlowRateInMs = 150;
        public static final int kTalonCANStatusVerySlowRateInMs = 255;  // 255 is max configurable period for CTRE CAN frames
        public static final int kTalonCANControlPrimaryRateInMs = 20;
        public static final int kTalonCANControlFollowerRateInMs = 40;
        public static final int kCANRetryCount = 10;
    }

    public static final class Swerve {
        // always ensure gyro is counter-clock-wise positive
        public static final boolean invertGyro = true;

        // drivetrain constants
        public static final double kTrackWidth = Units.inchesToMeters(22.50);
        public static final double kWheelBase = Units.inchesToMeters(22.50);
        public static final double kWheelDiameter = Units.inchesToMeters(4.00);
        public static final double kWheelCircumference = kWheelDiameter * Math.PI;

        // open loop (percent output) ramp probably better handled with our slew rate limiter
        public static final double kOpenLoopRamp = 0.0;
        public static final double kClosedLoopRamp = 0.0;

        // SDS Swerve Module MK3 fast drive gearing
        public static final double kDriveGearRatio = (6.86 / 1.0);
        public static final double kAngleGearRatio = (12.8 / 1.0);

        public static final double kDriveEncoderTicksPerRev = 2048.0 * kDriveGearRatio;  // 2048 ticks for Falcon 500
        public static final double kDriveEncoderDistancePerPulse =
            (kWheelDiameter * Math.PI) / (double) kDriveEncoderTicksPerRev;
        
        public static final double kAngleEncoderTicksPerRev = 2048.0 * kAngleGearRatio;
        public static final double kAngleEncoderTicksPerDegree = kAngleEncoderTicksPerRev / 360.0;
        public static final double kAngleEncoderDegreesPerTick = 360.0 / kAngleEncoderTicksPerRev;

        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

        // swerve current limiting
        public static final boolean kAngleEnableCurrentLimit = true;
        public static final int kAngleCurrentLimit = 20;
        public static final int kAngleThresholdCurrent = 30;
        public static final double kAngleThresholdDuration = 0.2;

        public static final boolean kDriveEnableCurrentLimit = true;
        public static final int kDriveCurrentLimit = 40;
        public static final int kDriveThresholdCurrent = 60;
        public static final double kDriveThresholdDuration = 0.5;

        // motion magic config values
        // 90 degree turn is 6553 ticks with 12.8 gear ratio.  Full output of Falcon with no load is ~6300rpm.
        // If we target 60% output, result is ~63 motor spindle rotations per second (~6.3 rotations per 100ms, or 12902 ticks/100ms).
        // Acceleration is units/100ms per second, so multiply by 20 to reach specified velocity in 50ms
        // note that angleKP needs to be high enough to reach the specified velocity
        public static final double kMotionCruiseVelocity = 12902;
        public static final double kMotionAcceleration = kMotionCruiseVelocity * 20;
        public static final int kMotionCurveStrength = 0;
        // when motion magic is used, what wheel angle delta is acceptable before adding drive power
        public static final double kAngleDeltaForDrive = 30.0;
        public static final double kAngleNeutralDeadband = 0.01;

        // angle motor PID values
        public static final double angleKP = 0.8;
        public static final double angleKI = 0.0;
        public static final double angleKD = 8.0;
        public static final double angleKF = 0.0;

        // angle motor PID values for simulation
        public static final double angleKPSim = 0.40;
        public static final double angleKISim = 0.0;
        public static final double angleKDSim = 0.00;
        public static final double angleKFSim = 0.0;

        // angle motor peak output
        public static final double maxSteerOutput = 1.0;

        // drive motor PID values
        public static final double driveKP = 0.10;//0.16;  // from sysid tool
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        // drive motor characterization values
        // use robot characterization tool (sysid) to determine these
        // it installs as part of WPILib suite. Also at https://github.com/wpilibsuite/sysid
        // also see https://www.chiefdelphi.com/t/applying-sysid-feedforward-to-an-sds-swerve-mk4/398455/8
        public static final double driveKS = 0.2;
        public static final double driveKV = 2.0;
        public static final double driveKA = 0.1;

        // max swerve speed and rotation
        public static final double maxSpeed = 4.5;          // max speed we want the robot to go in open loop mode
        public static final double maxFalconSpeed = 4.5;    // meters per second, theoretical max is 16.2 ft/s or 4.9 m/s
        public static final double maxAngularVelocity = 1.5 * (2 * Math.PI); // 1.5 rotations/s (radians)

        // angle and drive motor inverts
        public static final boolean invertDriveMotor = false;
        public static final boolean invertAngleMotor = false;

        // angle encoder invert
        public static final boolean canCoderInvert = false;


        // module specific constants

        // Modules on each side should have wheels consistent with each other.
        // In other words, bevel gear should be facing the same direction across all modules.
        // Adjust invertDriveMotor if forward/backward are reversed.

        // front left module - module 0
        public static final class Module0 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 20;
            public static final double angleOffset = 161.719;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, invertDriveMotor);
        }

        // front right module - module 1
        public static final class Module1 {
            public static final int driveMotorID = 12;
            public static final int angleMotorID = 13;
            public static final int canCoderID = 21;
            public static final double angleOffset = 227.02;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, invertDriveMotor);
        }
        
        // rear left module - module 2
        public static final class Module2 {
            public static final int driveMotorID = 14;
            public static final int angleMotorID = 15;
            public static final int canCoderID = 22;
            public static final double angleOffset = 282.217;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, invertDriveMotor);
        }

        // rear right module - module 3
        public static final class Module3 {
            public static final int driveMotorID = 16;
            public static final int angleMotorID = 17;
            public static final int canCoderID = 23;
            public static final double angleOffset = 258.047;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, invertDriveMotor);
        }
    }

    public static final class IntakeConstants {
        public static final int kIntakeArmMotorID = 1;
        public static final int kIntakeCanCoderID = 2;
        public static final int kIntakeSpinnerMotorID = 2;
        public static final int kIntakeArmCurrentLimit = 20;
        public static final double kIntakeSpinForwardSpeed = 0.6;
        public static final double kIntakeSpinBackwardSpeed = -0.3;
        public static final double kpIntakeRio = 0.010;
        public static final double kiIntakeRio = 0.0;
        public static final double kdIntakeRio = 0.0;
        // range is ~180 degrees
        public static final double kIntakeUpSensorDegrees = 1.0;//-3164;
        public static final double kIntakeDownSensorDegrees = 170;//-705;
        public static final double kIntakeArmMaxVelocityTicksPerS = 450.0;
        public static final double kIntakeArmMaxAccelTicksPerSSquared = 1700.0;
        public static final double kIntakeArmPositionTolerance = .12 * Math.abs(0 - kIntakeDownSensorDegrees);
    }

    public static final class KickerConstants {
        public static final int kKickerMotorID = 3;
        public static final double kForwardSpeed = 0.3;
        public static final double kBackwardSpeed = -0.2;
    }

    public static final class ConveyorConstants {
        public static final int kConveyorMotorID = 4;
        public static final double kIntakeSpeed = 0.20;
        public static final double kShootSpeed = 0.40;
        public static final double kBackwardSpeed = -0.20;
        public static final double kBackwardFastSpeed = -0.30;
    }

    public static final class ShooterConstants {
        public static final int kLowerDeviceID = 31;
        public static final int kUpperDeviceID = 32;

        public static final double kEncoderTicksPerRev = 2048;
        public static final double kUnitsPer100MSToRPMConversion = 600.0 / kEncoderTicksPerRev;
        public static final double kGearRatio = 1.0;
        public static final double kMomentOfInertia = 0.0007718;

        public static final double kVelocityTolerancePercent = 0.02;
    }

    public static final class ClimberConstants {
        public static final int kFalconID = 40;

        // TODO: update with correct values derived from sysid
        public static final double kClimbKP = 0.05;  // climb Kp value for Talon

        public static final double kClimbKS = 0.0;  // static volts
        public static final double kClimbKG = 0.0;  // gravity volts
        public static final double kClimbKV = 0.0;  // volts * seconds * distance

        public static final double kEncoderTicksPerRev = 2048;
        public static final double kClimbPositionTicksTop = 610000;
        public static final double kClimbPositionTicksBottom = 1000;
        public static final double kClimbPositionTicksZero = 0;
        public static final double kClimbPositionTicksHome = 6000;

        // output in percent (1.0 is 100%)
        public static final double kClimberClosedLoopPeakOutput = 1.0;
        public static final double kClimberManualPower = 1.0;
        public static final double kClimberZeroStatorCurrentThreshhold = 15.0;

        // limit current to certain amount if over threshhold for certain time
        public static final double kStatorCurrentLimit = 60.0;
        public static final double kStatorCurrentThreshhold = 50.0;
        public static final double kStatorCurrentThreshholdTime = 5.00;
        public static final boolean kStatorCurrentEnabled = true;

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Constants.Swerve.maxAngularVelocity;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 12 * Math.PI;
    
        // For a value of 1.0, the holonomic drive controller will add an additional meter per second for every meter of error
        public static final double kPXController = 2.0;//5;
        public static final double kPYController = 2.0;//5;
        public static final double kPThetaController = 3;//1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }

      public static final class TurnPIDConstants {
        public static final double kpTurnRio = 0.20;//0.18;//0.022;
        public static final double kiTurnRio = 0.000;
        public static final double kdTurnRio = 0.0;//0.0002;
      }

     

      public static final class LimelightVals {
        // https://firstfrc.blob.core.windows.net/frc2022/FieldAssets/2022FieldDrawings-RAPIDREACTSpecific.pdf
        // page 77, 179

        // https://firstfrc.blob.core.windows.net/frc2022/FieldAssets/2022LayoutMarkingDiagram.pdf

        // PhotonVision configured to use top of target as height
        // all TARGET_* constants are in inches (as described in game manual)
        public static final double TARGET_HEIGHT_OFF_GROUND = 102.62;   // center of tape (height)
        public static final double TARGET_HEIGHT = 2.0;
        // generally only a max of 4 pieces of reflective tape are visible
        public static final double TARGET_WIDTH = 5.45 + (2 * 4.91) + (2 * 5.04) + (2 * 4.18);
    
    
        public static final double CAMERA_HEIGHT = 30.00; // height of camera from ground in inches
        public static final double TESTING_TARGET_DISTANCE = 70; // temporary distance between camera and target for calc
                                                                  // the camera angle
        public static final double TESTING_CAMERA_ANGLE = 39.05; // temporary camera angle
    
        // target location on field in Pose2d format
        public static Pose2d targetPose2dLocation = new Pose2d(Units.inchesToMeters(324.0 - (53.38 / 2)), Units.inchesToMeters(162.0), new Rotation2d());

      }
    

}
