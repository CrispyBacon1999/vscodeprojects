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
public class ThreeBallRight extends SequentialCommandGroup {

  private Shooter m_shooter;
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private Intake m_intake;
  private Magazine m_magazine;

  /** Creates a new FiveBallRight. */
  public ThreeBallRight(
    Shooter shooter,
    DrivetrainSubsystem drivetrainSubsystem,
    Intake intake,
    Magazine magazine
  ) {
    m_shooter = shooter;
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_intake = intake;
    m_magazine = magazine;

    m_shooter.zeroEncoderOfHood();
    m_drivetrainSubsystem.zeroGyroscope();

    TrajectoryConfig trajectoryConfig = Constants.auto.follower.T_CONFIG;

    Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(new Translation2d(-.3, 0), new Translation2d(-.5, 0)),
      new Pose2d(-.7, 0, Rotation2d.fromDegrees(-10)),
      trajectoryConfig
    );

    Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(-.7, 0, new Rotation2d(-10)),
      List.of(new Translation2d(-.2, .5), new Translation2d(.1, .8)),
      new Pose2d(.2, 1.7, Rotation2d.fromDegrees(-55)), // .4 2.15 .35 1.75
      trajectoryConfig
    );

    PIDController xController = Constants.auto.follower.X_PID_CONTROLLER;
    PIDController yController = Constants.auto.follower.Y_PID_CONTROLLER;
    ProfiledPIDController thetaController =
      Constants.auto.follower.ROT_PID_CONTROLLER;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
      trajectory1,
      m_drivetrainSubsystem::getPose2d,
      m_drivetrainSubsystem.getKinematics(),
      xController,
      yController,
      thetaController,
      m_drivetrainSubsystem::setAllStates,
      m_drivetrainSubsystem
    );

    SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
      trajectory2,
      m_drivetrainSubsystem::getPose2d,
      m_drivetrainSubsystem.getKinematics(),
      xController,
      yController,
      thetaController,
      m_drivetrainSubsystem::setAllStates,
      m_drivetrainSubsystem
    );

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoPickUpBall(m_intake, m_magazine, m_shooter, 11500, -4)
      .alongWith(
          new SequentialCommandGroup(
            new InstantCommand(
              () ->
                m_drivetrainSubsystem.resetOdometry(
                  trajectory1.getInitialPose()
                )
            ),
            swerveControllerCommand1,
            new InstantCommand(() -> m_drivetrainSubsystem.stopModules()),
            new AutoShootCommand(m_magazine, m_shooter, 11500),
            new WaitCommand(2),
            new AutoShootCommand(m_magazine, m_shooter, 11500),
            new InstantCommand(
              () ->
                m_drivetrainSubsystem.resetOdometry(
                  trajectory2.getInitialPose()
                )
            ),
            swerveControllerCommand2,
            new InstantCommand(() -> m_drivetrainSubsystem.stopModules()),
            new WaitCommand(1),
            new AutoShootCommand(m_magazine, m_shooter, 11500)
          )
        )
    );
  }
}
