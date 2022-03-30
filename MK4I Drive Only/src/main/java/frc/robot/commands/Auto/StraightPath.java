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
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Shooter;
import java.util.List;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StraightPath extends SequentialCommandGroup {

  private Shooter m_shooter;
  private DrivetrainSubsystem m_drivetrainSubsystem;

  /** Creates a new StraightPath. */
  public StraightPath(
    Shooter shooter,
    DrivetrainSubsystem drivetrainSubsystem
  ) {
    m_shooter = shooter;
    m_drivetrainSubsystem = drivetrainSubsystem;

    m_shooter.zeroEncoderOfHood();
    m_drivetrainSubsystem.zeroGyroscope();

    TrajectoryConfig trajectoryConfig = Constants.auto.follower.T_CONFIG;

    Trajectory trajectoryStraight = TrajectoryGenerator.generateTrajectory(
      new Pose2d(-.7, 0, new Rotation2d(-10)),
      List.of(new Translation2d(-.2, .5), new Translation2d(.1, .8)),
      new Pose2d(.2, 1.7, Rotation2d.fromDegrees(-55)),
      trajectoryConfig
    );

    PIDController xController = Constants.auto.follower.X_PID_CONTROLLER;
    PIDController yController = Constants.auto.follower.Y_PID_CONTROLLER;
    ProfiledPIDController thetaController =
      Constants.auto.follower.ROT_PID_CONTROLLER;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(
        () ->
          m_drivetrainSubsystem.resetOdometry(
            trajectoryStraight.getInitialPose()
          )
      ),
      new SwerveControllerCommand(
        trajectoryStraight,
        m_drivetrainSubsystem::getPose2d,
        m_drivetrainSubsystem.getKinematics(),
        xController,
        yController,
        thetaController,
        m_drivetrainSubsystem::setAllStates,
        m_drivetrainSubsystem
      )
    );
  }
}
