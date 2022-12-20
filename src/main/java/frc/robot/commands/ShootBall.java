// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.KickerConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootBall extends SequentialCommandGroup {
    /** Creates a new ShootBall. */
    public ShootBall(SwerveDrive drive, Conveyor conveyor, Kicker kicker, Shooter shooter, LimelightSubsystem limelight, boolean bLower) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());

        addCommands(
            // use .until(condition), .withTimeout(time), and .andThen() as necessary
            
            // spin up shooter
/*            new ParallelCommandGroup(new TurnToTarget(drive, limelight),
                                     new ShootWithVision(limelight, shooter, bLower)),
            new SequentialCommandGroup(
                new RunConveyor(conveyor, ConveyorConstants.kBackwardSpeed).withTimeout(0.1),
                new RunKicker(kicker, KickerConstants.kBackwardSpeed).withTimeout(0.1),
                // wait until shooter has spun up
                new WaitForShooterRPMAndTarget(shooter, limelight),
                // stop conveyor and pulse kicker
                new ParallelCommandGroup(new RunConveyor(conveyor, 0.0).withTimeout(0.2),
                                         new RunKicker(kicker, KickerConstants.kForwardSpeed).withTimeout(0.2)),
                new RunKicker(kicker, 0.0).until(shooter::isAtDesiredVelocity),
                new ParallelCommandGroup(new RunConveyor(conveyor, ConveyorConstants.kShootSpeed).withTimeout(0.4),
                                         // spit out second ball
                                         new RunKicker(kicker, KickerConstants.kForwardSpeed).withTimeout(0.4))
                
            )
*/
            new TurnToTarget(drive, limelight).withTimeout(1.5),
            new ParallelRaceGroup(
                new ShootWithVision(limelight, shooter, bLower),
                new SequentialCommandGroup(
                    new RunConveyor(conveyor, ConveyorConstants.kBackwardSpeed).withTimeout(0.1),
                    new RunKicker(kicker, KickerConstants.kBackwardSpeed).withTimeout(0.1),
                    // wait until shooter has spun up. Consider measurement rather than timeout as condition
                    new WaitCommand(.1),
                    // stop conveyor and pulse kicker
                    new ParallelCommandGroup(
                        new RunConveyor(conveyor, 0.0).withTimeout(0.2),
                        new RunKicker(kicker, KickerConstants.kForwardSpeed).withTimeout(0.2)
                    ),
                    new RunKicker(kicker, 0.0).withTimeout(0.1),
                    new ParallelCommandGroup(
                        new RunConveyor(conveyor, ConveyorConstants.kShootSpeed).withTimeout(0.4),
                        // spit out second ball
                        new RunKicker(kicker, KickerConstants.kForwardSpeed).withTimeout(0.4)
                    )
                )
            )
        );
    }
}
