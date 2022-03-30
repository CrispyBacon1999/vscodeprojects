// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class TurnToHub extends CommandBase {
  DrivetrainSubsystem m_drivetrain;
  Limelight m_limelight;
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;

  private Translation2d hub_position = new Translation2d(Constants.HUB_CENTER_X, Constants.HUB_CENTER_Y);
  private double kP = 1;

  /** Creates a new TurnToHub. */
  public TurnToHub(DrivetrainSubsystem drivetrain, Limelight limelight, DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier) {
    m_drivetrain = drivetrain;
    m_limelight = limelight;
    m_translationXSupplier = translationXSupplier;
    m_translationYSupplier = translationYSupplier;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robot_pose = m_drivetrain.getFieldPose2d();
    Translation2d robot_position = robot_pose.getTranslation();

    // Radian angle
    double angle = Math.atan2(robot_position.getY() - hub_position.getY(), robot_position.getX() - hub_position.getX());
    double angle_error = angle - m_drivetrain.getGyroscopeRotation().getRadians();

    // Respect x and y movement, but ignore driver rotation
    m_drivetrain.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            m_translationXSupplier.getAsDouble(), m_translationYSupplier.getAsDouble(),
            Math.max(-1, Math.min(angle_error, 1)) * kP * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            m_drivetrain.getGyroscopeRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  @Override
  public boolean isFinished() {
    return m_limelight.isTargeting() && m_limelight.ifValidTarget() > 0;
  }

}
