// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoPickUpBall;
import frc.robot.commands.AutoShootCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import java.util.List;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallRight extends SequentialCommandGroup {

  private Shooter m_shooter;
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private Intake m_intake;
  private Magazine m_magazine;

  /** Creates a new FiveBallRight. */
  public FiveBallRight(
      Shooter shooter,
      DrivetrainSubsystem drivetrainSubsystem,
      Intake intake,
      Magazine magazine) {
    m_shooter = shooter;
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_intake = intake;
    m_magazine = magazine;

    m_shooter.zeroEncoderOfHood();
    m_drivetrainSubsystem.zeroGyroscope();

    // Set field pose position to where robot is set up (generally)
    m_drivetrainSubsystem.setFieldPosition(new Translation2d(7.8, 6), new Rotation2d(-Math.PI / 4));

    TrajectoryConfig trajectoryConfig = Constants.auto.follower.T_CONFIG;

    Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(-.3, 0), new Translation2d(-.5, 0)),
        new Pose2d(-.7, 0, Rotation2d.fromDegrees(-10)),
        trajectoryConfig);

    Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(-.7, 0, new Rotation2d(-10)),
        List.of(new Translation2d(-.2, .5), new Translation2d(.1, .8)),
        new Pose2d(.2, 1.7, Rotation2d.fromDegrees(-55)), // .4 2.15 .35 1.75
        trajectoryConfig);

    Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(-65)), // .35 1.75
        List.of(
            new Translation2d(-.9 * .3, 2.5 * .3),
            new Translation2d(-.9 * .6, 2.5 * .6)),
        // Closer to tower decrease 5.5
        // Closer to middle of field increase -.5
        // 5.6 worked then ran into wall I suggest lowering it by 1 and tuning from
        // there
        new Pose2d(-0.9, 2.5, Rotation2d.fromDegrees(-65)),
        trajectoryConfig);

    PIDController xController = Constants.auto.follower.X_PID_CONTROLLER;
    PIDController yController = Constants.auto.follower.Y_PID_CONTROLLER;
    ProfiledPIDController thetaController = Constants.auto.follower.ROT_PID_CONTROLLER;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
        trajectory1,
        m_drivetrainSubsystem::getPose2d,
        m_drivetrainSubsystem.getKinematics(),
        xController,
        yController,
        thetaController,
        m_drivetrainSubsystem::setAllStates,
        m_drivetrainSubsystem);

    SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
        trajectory2,
        m_drivetrainSubsystem::getPose2d,
        m_drivetrainSubsystem.getKinematics(),
        xController,
        yController,
        thetaController,
        m_drivetrainSubsystem::setAllStates,
        m_drivetrainSubsystem);

    SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(
        trajectory3,
        m_drivetrainSubsystem::getPose2d,
        m_drivetrainSubsystem.getKinematics(),
        xController,
        yController,
        thetaController,
        m_drivetrainSubsystem::setAllStates,
        m_drivetrainSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelRaceGroup(
            new AutoPickUpBall(m_intake, m_magazine, m_shooter, 11500, -4),
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> m_drivetrainSubsystem.resetOdometry(trajectory1.getInitialPose())),
                swerveControllerCommand1,
                new InstantCommand(() -> m_drivetrainSubsystem.stopModules()),
                new AutoShootCommand(m_magazine, m_shooter, 11500),
                new WaitCommand(1),
                new AutoShootCommand(m_magazine, m_shooter, 11500),
                new InstantCommand(
                    () -> m_drivetrainSubsystem.resetOdometry(trajectory2.getInitialPose())),
                swerveControllerCommand2,
                new InstantCommand(() -> m_drivetrainSubsystem.stopModules()),
                new WaitCommand(1),
                new AutoShootCommand(m_magazine, m_shooter, 11500)))
                    .andThen(
                        new AutoPickUpBall(m_intake, m_magazine, m_shooter, 16000, -8.2)
                            .alongWith(
                                new SequentialCommandGroup(
                                    new InstantCommand(
                                        () -> m_drivetrainSubsystem.resetOdometry(
                                            trajectory3.getInitialPose())),
                                    swerveControllerCommand3,
                                    new InstantCommand(() -> m_drivetrainSubsystem.stopModules()),
                                    new WaitCommand(1),
                                    new AutoShootCommand(m_magazine, m_shooter, 16000),
                                    new WaitCommand(1),
                                    new AutoShootCommand(m_magazine, m_shooter, 16000)))));
  }
}
