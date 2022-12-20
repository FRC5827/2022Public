// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeSpinner;
import frc.robot.subsystems.LimelightSubsystem;
//import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.IntakeArm.IntakeState;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;

import frc.lib.util.TrajectoryLoader;
import frc.lib.util.TrajectoryLoader.pickTrajectory;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Controllers
    private final XboxController m_driverInput = new XboxController(0);

    // Drive Controls
    private final int m_translationAxis = XboxController.Axis.kLeftY.value;
    private final int m_strafeAxis = XboxController.Axis.kLeftX.value;
    private final int m_rotationAxis = XboxController.Axis.kRightX.value;

    // Driver Buttons
    private final JoystickButton m_zeroGyro = new JoystickButton(m_driverInput, XboxController.Button.kLeftBumper.value);
    private final Trigger m_spinIn = new Trigger(() -> { return m_driverInput.getLeftTriggerAxis() > 0.7;});
    private final JoystickButton m_spinOut = new JoystickButton(m_driverInput, XboxController.Button.kX.value);
    private final JoystickButton m_shootLow = new JoystickButton(m_driverInput, XboxController.Button.kB.value);
    private final Trigger m_shootHigh = new Trigger(() -> { return m_driverInput.getRightTriggerAxis() > 0.7;});
    private final JoystickButton m_drivePointingToTarget = new JoystickButton(m_driverInput, XboxController.Button.kY.value);
    private final JoystickButton m_manualClimbUp = new JoystickButton(m_driverInput, XboxController.Button.kStart.value);
    private final JoystickButton m_manualClimbDown = new JoystickButton(m_driverInput, XboxController.Button.kBack.value);
    private final JoystickButton m_fieldRelativeRightBumper = new JoystickButton(m_driverInput, XboxController.Button.kRightBumper.value);
    private final POVButton m_intakeUp = new POVButton(m_driverInput, 270);
    private final POVButton m_intakeDown = new POVButton(m_driverInput, 90);
    private final POVButton m_climbUp = new POVButton(m_driverInput, 0);
    private final POVButton m_climbDown = new POVButton(m_driverInput, 180);
   


    // Subsystems
    private final SwerveDrive m_swerveDrive = new SwerveDrive(this);
    private final Shooter m_shooter = new Shooter();
    private final Kicker m_kicker = new Kicker();
    private final IntakeArm m_intakeArm = new IntakeArm(this);
    private final IntakeSpinner m_intakeSpinner = new IntakeSpinner();
    private final Conveyor m_conveyor = new Conveyor();
    private final Climber m_climber = new Climber();
    private final LimelightSubsystem m_limelight = new LimelightSubsystem();
    //private final PhotonVisionSubsystem m_photonVision = new PhotonVisionSubsystem();

    // auto selector for dashboard
    private SendableChooser<Double> m_autoDelay;
    private SendableChooser<Double> m_humanPlayerDelay;
	private SendableChooser<pickTrajectory> m_autoSelector;

    // Definitions of the various different autonomous commands go here
	public AutonomousCommand m_1_BaseCollect;
	public AutonomousCommand m_2_MidRoam;
	public AutonomousCommand m_3_BaseCollect;
	public AutonomousCommand m_3_BotRoam;
	public AutonomousCommand m_4_BotCollect;

    private boolean m_fieldRelative;
    private boolean m_openLoop;
    private ShuffleboardTab m_driveShuffleboardTab;
    private NetworkTableEntry m_fieldRelativeStateShuffleboard;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // Default for WPILib is to output telemetry for all internal objects, which can be costly
        // from a memory allocation aspect and CPU time due to garbage collection,
        // plus consumes bandwidth.
        LiveWindow.disableAllTelemetry();

   		// Create camera server for usb camera
		UsbCamera camera = CameraServer.startAutomaticCapture();
		if (RobotBase.isReal()) { 
			// reduce USB and network bandwidth usage
            camera.setResolution(160, 120);
			camera.setFPS(20);
		}

        // Set the scheduler to log Shuffleboard events for command initialize, interrupt, finish
        CommandScheduler.getInstance()
            .onCommandInitialize(
                command ->
                    Shuffleboard.addEventMarker(
                        "Command initialized", command.getName(), EventImportance.kNormal));
        CommandScheduler.getInstance()
            .onCommandInterrupt(
                command ->
                    Shuffleboard.addEventMarker(
                        "Command interrupted", command.getName(), EventImportance.kNormal));
        CommandScheduler.getInstance()
            .onCommandFinish(
                command ->
                    Shuffleboard.addEventMarker(
                        "Command finished", command.getName(), EventImportance.kNormal));


        m_fieldRelative = true;
        m_openLoop = true;
                
        m_driveShuffleboardTab = Shuffleboard.getTab("Drivetrain");
        m_fieldRelativeStateShuffleboard = m_driveShuffleboardTab.add("Field Relative", m_fieldRelative)
                                                    .withPosition(0, 0)
                                                    .withSize(1, 1)
                                                    .withWidget(BuiltInWidgets.kToggleButton).getEntry();
        m_driveShuffleboardTab.add("FieldRelativeStatus", m_fieldRelative)
                                                    .withPosition(1, 0)
                                                    .withSize(1, 1)
                                                    .withWidget(BuiltInWidgets.kBooleanBox).getEntry();

                

        // Configure the button bindings
        configureButtonBindings();

        m_swerveDrive.setDefaultCommand(
            new TeleopSwerve(m_swerveDrive, m_driverInput, m_translationAxis, m_strafeAxis, m_rotationAxis, m_openLoop));

        //set up default commands for each subsystem
        m_conveyor.setDefaultCommand(new RunCommand(() -> m_conveyor.activate(0.0), m_conveyor));
        m_shooter.setDefaultCommand(new ShooterSetVoltage(m_shooter, 0, 0));

        // init chooser for dashboard selections
        initChooser();

        // Generating trajectories can be compute intensive but only happens once at startup when RobotContainer
		// is instantiated in robotInit(), so any delay can be absorbed
		TrajectoryLoader.loadTrajectories();

        resetIntakeArmEncoder();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Driver Buttons
        m_fieldRelativeRightBumper.whenPressed(new InstantCommand(this::toggleFieldRelative));
        m_zeroGyro.whenPressed(new InstantCommand(() -> m_swerveDrive.zeroHeading()));
        m_spinIn.whileActiveOnce(new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown)
                                     .alongWith(new IntakeSpin(m_intakeSpinner, m_conveyor, m_kicker, m_swerveDrive,true)));

        m_spinIn.whenInactive(new RunConveyor(m_conveyor, ConveyorConstants.kIntakeSpeed).withTimeout(1.0));
        m_spinOut.whenHeld(new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown)
                               .alongWith(new IntakeSpin(m_intakeSpinner, m_conveyor, m_kicker, m_swerveDrive,false)));
        m_shootLow.whenPressed((new ShootBall(m_swerveDrive, m_conveyor, m_kicker, m_shooter, m_limelight, true))
                                    .withInterrupt(() -> (Math.abs(m_driverInput.getRawAxis(m_rotationAxis)) > .20) ||
                                                          Math.abs(m_driverInput.getRawAxis(m_translationAxis)) > .20 ||
                                                          Math.abs(m_driverInput.getRawAxis(m_strafeAxis)) > .20));
    
        m_shootHigh.whenActive((new ShootBall(m_swerveDrive, m_conveyor, m_kicker, m_shooter, m_limelight, false))
                                    .withInterrupt(() -> (Math.abs(m_driverInput.getRawAxis(m_rotationAxis)) > .20) ||
                                                          Math.abs(m_driverInput.getRawAxis(m_translationAxis)) > .20 ||
                                                          Math.abs(m_driverInput.getRawAxis(m_strafeAxis)) > .20));
                  
        m_drivePointingToTarget.whenPressed(
            new DrivePointingToTarget(m_swerveDrive, m_limelight, m_driverInput,
              m_translationAxis, m_strafeAxis, m_rotationAxis, m_fieldRelative)
                .withInterrupt(() -> Math.abs(m_driverInput.getRawAxis(m_rotationAxis)) > .20));

        m_intakeUp.whenPressed(new IntakeUpDown(m_intakeArm, IntakeState.IntakeUp));
        m_intakeDown.whenPressed(new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown));

        m_climbUp.whenPressed(new MoveClimberToPosition(m_climber, ClimberConstants.kClimbPositionTicksTop));
        m_climbDown.whenPressed(new MoveClimberToPosition(m_climber, ClimberConstants.kClimbPositionTicksHome));

        // Manual Retraction and Extension with Different Buttons
        m_manualClimbUp.whenHeld(new MoveClimber(m_climber, ClimberConstants.kClimberManualPower));
        m_manualClimbDown.whenHeld(new MoveClimber(m_climber, -ClimberConstants.kClimberManualPower));

    }

    /**
     * We don't want this command to run during autonomous, so call from robot.java when teleop initializes
     */
    public void teleopInit() {
        m_intakeArm.setDefaultCommand(new PerpetualCommand(new IntakeUpDown(m_intakeArm, IntakeState.IntakeUp)));
        CommandScheduler.getInstance().schedule(new InitClimber(m_climber)
                                                .andThen(new MoveClimberToPosition(m_climber, ClimberConstants.kClimbPositionTicksHome)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

		switch (m_autoSelector.getSelected()) {

            case _1_TopBaseCollect:
                return new AutonomousCommand(List.of(
                                                new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                                                    new Pose2d(
                                                        TrajectoryLoader._1_TopBaseCollect.getInitialState().poseMeters.getX(),
                                                        TrajectoryLoader._1_TopBaseCollect.getInitialState().poseMeters.getY(),
                                                        TrajectoryLoader._1_TopBaseCollect.getInitialState().holonomicRotation),
                                                    false)),
                                                new WaitCommandDashboard(m_autoDelay),
                                                new ParallelCommandGroup(new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown),
                                                                         new ParallelRaceGroup(new IntakeSpin(m_intakeSpinner, m_conveyor, m_kicker, m_swerveDrive, true),
                                                                                               this.getFollowPathCommand(TrajectoryLoader._1_TopBaseCollect))),
                                                new ShootBall(m_swerveDrive, m_conveyor, m_kicker, m_shooter, m_limelight, false),
                                                new ParallelRaceGroup(new IntakeSpin(m_intakeSpinner, m_conveyor, m_kicker, m_swerveDrive,true),
                                                                         this.getFollowPathCommand(TrajectoryLoader._1_TopBaseCollectPart2)),
                                                new ShootBall(m_swerveDrive, m_conveyor, m_kicker, m_shooter, m_limelight, false))
                                            );
            case _1_ShootTaxi:
                return new AutonomousCommand(List.of(
                                                new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                                                    new Pose2d(
                                                        TrajectoryLoader._1_ShootTaxi.getInitialState().poseMeters.getX(),
                                                        TrajectoryLoader._1_ShootTaxi.getInitialState().poseMeters.getY(),
                                                        TrajectoryLoader._1_ShootTaxi.getInitialState().holonomicRotation),
                                                    false)),
                                                new ShootBall(m_swerveDrive, m_conveyor, m_kicker, m_shooter, m_limelight, false),
                                                new WaitCommandDashboard(m_autoDelay),
                                                this.getFollowPathCommand(TrajectoryLoader._1_ShootTaxi))
                                            );

            case _1_TwoBallTaxi:
                return new AutonomousCommand(List.of(
                                                new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                                                    new Pose2d(
                                                        TrajectoryLoader._1_TwoBallTaxi.getInitialState().poseMeters.getX(),
                                                        TrajectoryLoader._1_TwoBallTaxi.getInitialState().poseMeters.getY(),
                                                        TrajectoryLoader._1_TwoBallTaxi.getInitialState().holonomicRotation),
                                                    false)),
                                                new WaitCommandDashboard(m_autoDelay),
                                                new ParallelCommandGroup(new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown),
                                                                         new ParallelRaceGroup(new IntakeSpin(m_intakeSpinner, m_conveyor, m_kicker, m_swerveDrive, true),
                                                                                               this.getFollowPathCommand(TrajectoryLoader._1_TwoBallTaxi))),
                                                new ShootBall(m_swerveDrive, m_conveyor, m_kicker, m_shooter, m_limelight, false))
                                            );

            case _2_ShootAndTaxi:
                return new AutonomousCommand(List.of(
                                                new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                                                    new Pose2d(
                                                        TrajectoryLoader._2_ShootAndTaxi.getInitialState().poseMeters.getX(),
                                                        TrajectoryLoader._2_ShootAndTaxi.getInitialState().poseMeters.getY(),
                                                        TrajectoryLoader._2_ShootAndTaxi.getInitialState().holonomicRotation),
                                                    false)),
                                                new ShootBall(m_swerveDrive, m_conveyor, m_kicker, m_shooter, m_limelight, false),
                                                new WaitCommandDashboard(m_autoDelay),
                                                this.getFollowPathCommand(TrajectoryLoader._2_ShootAndTaxi))
                                            );

            case _3_TwoBallTaxi:
                return new AutonomousCommand(List.of(
                                                new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                                                    new Pose2d(
                                                        TrajectoryLoader._3_TwoBallTaxi.getInitialState().poseMeters.getX(),
                                                        TrajectoryLoader._3_TwoBallTaxi.getInitialState().poseMeters.getY(),
                                                        TrajectoryLoader._3_TwoBallTaxi.getInitialState().holonomicRotation),
                                                    false)),
                                                new WaitCommandDashboard(m_autoDelay),
                                                new ParallelCommandGroup(new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown),
                                                                         new ParallelRaceGroup(new IntakeSpin(m_intakeSpinner, m_conveyor, m_kicker, m_swerveDrive, true),
                                                                                               this.getFollowPathCommand(TrajectoryLoader._3_TwoBallTaxi))),
                                                new ShootBall(m_swerveDrive, m_conveyor, m_kicker, m_shooter, m_limelight, false))
                                            );
            case _4_BotBaseCollect:
                return new AutonomousCommand(List.of(
                                                new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                                                    new Pose2d(
                                                        TrajectoryLoader._4_BotBaseCollect.getInitialState().poseMeters.getX(),
                                                        TrajectoryLoader._4_BotBaseCollect.getInitialState().poseMeters.getY(),
                                                        TrajectoryLoader._4_BotBaseCollect.getInitialState().holonomicRotation),
                                                    false)),
                                                new WaitCommandDashboard(m_autoDelay),
                                                new ParallelCommandGroup(new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown),
                                                                         new ParallelRaceGroup(new IntakeSpin(m_intakeSpinner, m_conveyor, m_kicker, m_swerveDrive, true),
                                                                                               this.getFollowPathCommand(TrajectoryLoader._4_BotBaseCollect))),
                                                new ShootBall(m_swerveDrive, m_conveyor, m_kicker, m_shooter, m_limelight, false),
                                                new ParallelRaceGroup(new IntakeSpin(m_intakeSpinner, m_conveyor, m_kicker, m_swerveDrive, true),
                                                                         this.getFollowPathCommand(TrajectoryLoader._4_BotBaseCollectPart2)),
                                                new ShootBall(m_swerveDrive, m_conveyor, m_kicker, m_shooter, m_limelight, false))

                                            );

            case _4_TwoBallTaxi:
                return new AutonomousCommand(List.of(
                                                new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                                                    new Pose2d(
                                                        TrajectoryLoader._4_TwoBallTaxi.getInitialState().poseMeters.getX(),
                                                        TrajectoryLoader._4_TwoBallTaxi.getInitialState().poseMeters.getY(),
                                                        TrajectoryLoader._4_TwoBallTaxi.getInitialState().holonomicRotation),
                                                    false)),
                                                new WaitCommandDashboard(m_autoDelay),
                                                new ParallelCommandGroup(new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown),
                                                                         new ParallelRaceGroup(new IntakeSpin(m_intakeSpinner, m_conveyor, m_kicker, m_swerveDrive, true),
                                                                                               this.getFollowPathCommand(TrajectoryLoader._4_TwoBallTaxi))),
                                                new ShootBall(m_swerveDrive, m_conveyor, m_kicker, m_shooter, m_limelight, false))
                                            );


            case _4_FiveBall:
                return new AutonomousCommand(List.of(
                                               new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                                                    new Pose2d(
                                                        TrajectoryLoader._4_FiveBallPart1.getInitialState().poseMeters.getX(),
                                                        TrajectoryLoader._4_FiveBallPart1.getInitialState().poseMeters.getY(),
                                                        TrajectoryLoader._4_FiveBallPart1.getInitialState().holonomicRotation),
                                                    false)),
                                                new WaitCommandDashboard(m_autoDelay),
                                                new ParallelCommandGroup(new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown),
                                                                         new ParallelRaceGroup(new IntakeSpin(m_intakeSpinner, m_conveyor, m_kicker, m_swerveDrive, true),
                                                                                               this.getFollowPathCommand(TrajectoryLoader._4_FiveBallPart1))),
                                                new ShootBall(m_swerveDrive, m_conveyor, m_kicker, m_shooter, m_limelight, false),
                                                new ParallelRaceGroup(new IntakeSpin(m_intakeSpinner, m_conveyor, m_kicker, m_swerveDrive, true),
                                                                         this.getFollowPathCommand(TrajectoryLoader._4_FiveBallPart2)),
                                                new IntakeSpin(m_intakeSpinner, m_conveyor, m_kicker, m_swerveDrive, true).withTimeout(0.5),
                                                new ShootBall(m_swerveDrive, m_conveyor, m_kicker, m_shooter, m_limelight, false),
                                                new ParallelRaceGroup(new IntakeSpin(m_intakeSpinner, m_conveyor, m_kicker, m_swerveDrive, true),
                                                                         this.getFollowPathCommand(TrajectoryLoader._4_FiveBallPart3)),
                                                new ParallelRaceGroup(new WaitCommandDashboard(m_humanPlayerDelay), new IntakeSpin(m_intakeSpinner, m_conveyor, m_kicker,  m_swerveDrive, true)),
                                                //new InstantCommand(() -> System.out.println("5 ball pose: " + m_swerveDrive.getPose())),
                                                new ParallelRaceGroup(new IntakeSpin(m_intakeSpinner, m_conveyor, m_kicker, m_swerveDrive, true),
                                                                         this.getFollowPathCommand(TrajectoryLoader._4_FiveBallPart4)),
                                                new ShootBall(m_swerveDrive, m_conveyor, m_kicker, m_shooter, m_limelight, false))
                                            );

            default:
                // nothing to do -- shouldn't ever happen
                System.out.println("No auton command to run!");
                return new WaitCommand(0.0);
        }

    }

    
    /**
     * Used to reset gyro heading and pose/odometry.  Useful at beginning of autononmous period,
     * particularly if the robot has been sitting powered on for some time as gyro may have drifted.
     *  
     * @param pose to reset to
     */
    public void resetHeadingAndOdometry(Pose2d pose) {
        m_swerveDrive.resetOdometryAndAngleOffset(pose, true);
    }

    /**
     * Reset the intake encoder for relative position measurement.  Arm should be up when called.
     */
    public void resetIntakeArmEncoder() {
        m_intakeArm.resetIntakeArmEncoder();
    }

    /**
     * Used to set brake or neutral mode for motors on drive subsystem
     * 
     * @param mode if true, enable brake mode
     */
    public void setBrakeMode(boolean mode) {
        m_swerveDrive.setBrakeMode(mode);
    }

    /**
     * Toggles field relative state in networktables via shuffleboard boolean
     * for use by other commands/subsystems
     */
    public void toggleFieldRelative() {
        this.m_fieldRelative = !this.m_fieldRelative;
        m_fieldRelativeStateShuffleboard.setBoolean(this.m_fieldRelative);
    }

    /**
     * Returns boolean from vision processor if there is a valid target
     * @return true if there is a valid target
     */
    public boolean hasVisionTarget() {
        return m_limelight.hasValidTarget();
    }

    /**
     * Returns distance to the vision target.  Call hasTarget() first to ensure
     * there is a valid target or result could be invalid.
     * @return Distance to target in inches
     */
    public double getVisionDistanceToTarget() {
        return m_limelight.getDistance();
    }

    /**
     * Returns yaw offset in degrees of the vision target.  Call hasTarget() first to ensure
     * there is a valid target or result could be invalid.
     * @return Yaw offset of target
     */
    public double getVisionYawToTarget() {
        return m_limelight.getTX();
    }

    public void initChooser() {
        m_autoSelector = new SendableChooser<pickTrajectory>();
        m_autoSelector.addOption("1 - 4 ball terminal", pickTrajectory._1_TopBaseCollect);
        m_autoSelector.addOption("1 - (facing hub) 1 ball and taxi", pickTrajectory._1_ShootTaxi);
        m_autoSelector.addOption("1 - 2 ball and taxi", pickTrajectory._1_TwoBallTaxi);
        m_autoSelector.addOption("2 - (facing hub) 1 ball and taxi", pickTrajectory._2_ShootAndTaxi);
        m_autoSelector.addOption("3 - 2 ball and taxi", pickTrajectory._3_TwoBallTaxi);
        m_autoSelector.setDefaultOption("4 - 4 ball terminal", pickTrajectory._4_BotBaseCollect);
        m_autoSelector.addOption("4 - 2 ball and taxi", pickTrajectory._4_TwoBallTaxi);
        m_autoSelector.addOption("4 - 5 ball terminal", pickTrajectory._4_FiveBall);
		Shuffleboard.getTab("Autonomous").add("Autonomous sequence", m_autoSelector).withPosition(0, 0).withSize(2, 1);

        m_autoDelay = new SendableChooser<Double>();
		m_autoDelay.setDefaultOption("0 second delay", 0.0);
		m_autoDelay.addOption("2 second delay", 2.0);
		m_autoDelay.addOption("5 second delay", 5.0);
		m_autoDelay.addOption("8 second delay", 8.0);
		m_autoDelay.addOption("10 second delay", 10.0);
		Shuffleboard.getTab("Autonomous").add("Delay before auton", m_autoDelay).withPosition(2, 0).withSize(2, 1);

        m_humanPlayerDelay = new SendableChooser<Double>();
        m_humanPlayerDelay.addOption("0.8 second delay", 0.80);
        m_humanPlayerDelay.setDefaultOption("1 second delay", 1.0);
		m_humanPlayerDelay.addOption("2 second delay", 2.0);
		m_humanPlayerDelay.addOption("3 second delay", 3.0);
		Shuffleboard.getTab("Autonomous").add("Delay during auton", m_humanPlayerDelay).withPosition(2, 1).withSize(2, 1);

        Shuffleboard.getTab("Autonomous").add(new InstantCommand(m_swerveDrive::zeroModuleAngles).withName("Zero swerve angles"));
        //Shuffleboard.getTab("Autonomous").add(new InitClimber(m_climber).withName("Init Climber"));
        //Shuffleboard.getTab("Autonomous").add(new MoveClimberToPosition(m_climber, 250000).withName("Home Climber"));

    }

    public Command getFollowPathCommand(PathPlannerTrajectory trajectory) {
        ProfiledPIDController thetaController = new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
            trajectory,
            m_swerveDrive::getPose,
            Constants.Swerve.kSwerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            m_swerveDrive::setModuleStates,
            m_swerveDrive);

        return swerveCommand.andThen(() -> m_swerveDrive.drive(0.0, 0.0, 0.0, false, false));

    }
}
