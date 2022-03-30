// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.extra_libraries.PathPlanner;
import frc.robot.extra_libraries.PathPlannerTrajectory;
import frc.robot.extra_libraries.PathPlannerTrajectory.PathPlannerState;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RunBasicTrajectory extends CommandBase {

  private Pose2d currentPosition;
  private DrivetrainSubsystem m_drivetrain;
  private PathPlannerTrajectory target;
  private ChassisSpeeds speeds = new ChassisSpeeds();
  private ProfiledPIDController rot_pid;
  private PathPlannerState state;
  private HolonomicDriveController hController;
  private boolean commandFinished = false;
  private final Timer timer = new Timer();

  public RunBasicTrajectory(DrivetrainSubsystem m_drivetrain, String path) {
    this.m_drivetrain = m_drivetrain;
    rot_pid = Constants.auto.follower.ROT_PID_CONTROLLER;
    target =
      PathPlanner.loadPath(
        path,
        Constants.swerve.MAX_VEL_METERS,
        Constants.swerve.MAX_ANG_ACCEL
      );

    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    //System.out.println("Heading before"+m_drivetrain.getGyroscopeObj().getFusedHeading());
    //System.out.println("VELOCITY METERS: " + Constants.swerve.MAX_VEL_METERS);;
    //System.out.println("INITIAL POSE" + target.getInitialPose());
    m_drivetrain
      .getGyroscopeObj()
      .setYaw(target.getInitialState().poseMeters.getRotation().getDegrees());
    m_drivetrain.resetOdometry(target.getInitialPose());

    //System.out.println("Degrees: "+m_drivetrain.getPose2d().getRotation().getDegrees());

    //System.out.println("INITIAL POSE: " + target.getInitialPose().getRotation().getDegrees());
    //System.out.println("INITIAL POSE ROBOT = " + m_drivetrain.getGyroscopeRotation());

    //System.out.println("Heading after"+m_drivetrain.getGyroscopeObj().getFusedHeading());
    rot_pid.enableContinuousInput(-Math.PI, Math.PI);
    hController =
      new HolonomicDriveController(
        Constants.auto.follower.X_PID_CONTROLLER,
        Constants.auto.follower.Y_PID_CONTROLLER,
        Constants.auto.follower.ROT_PID_CONTROLLER
      );
    timer.reset();
    //System.out.println("Degrees: "+m_drivetrain.getPose2d().getRotation().getDegrees());

    //System.out.println("INITIAL POSE: " + target.getInitialPose().getRotation().getDegrees());
    timer.start();
  }

  @Override
  public void execute() {
    var curTime = timer.get();
    //System.out.println(curTime);
    state = (PathPlannerState) target.sample(curTime);
    currentPosition = m_drivetrain.getPose2d();
    speeds =
      hController.calculate(currentPosition, state, state.holonomicRotation);
    m_drivetrain.setAllStates(
      m_drivetrain.getKinematics().toSwerveModuleStates(speeds)
    );
    //System.out.println("Heading"+m_drivetrain.getGyroscopeObj().getFusedHeading());
    // String Y = Double.toString(currentPosition.getY());
    // String X = Double.toString(currentPosition.getX());
    //System.out.println("y= " + Y.substring(0,3));
    //System.out.println("X= "+X.substring(0,3));
    //System.out.println("State= "+ state);

    //System.out.println("speed"+state.velocityMetersPerSecond);
    //System.out.println(state.poseMeters.getX());
    if (curTime > 1.9) {
      commandFinished = true;
    }
  }

  /**
   * @return boolean
   */
  @Override
  public boolean isFinished() {
    return commandFinished;
  }

  /**
   * @param interrupted
   */
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    m_drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    //m_drivetrain.defense();
  }
}
