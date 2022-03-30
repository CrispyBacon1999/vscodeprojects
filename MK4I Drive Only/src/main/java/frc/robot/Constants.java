// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /**
   * The left-to-right distance between the drivetrain wheels
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = .749; // FIXME Measure and set trackwidth
  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_WHEELBASE_METERS = .749; // FIXME Measure and set wheelbase

  public static final int DRIVETRAIN_PIGEON_ID = 13; // FIXME Set Pigeon ID

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; // FIXME Set front left module drive motor ID
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; // FIXME Set front left module steer motor ID
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9; // FIXME Set front left steer encoder ID
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(
    188.7
  ); // FIXME Measure and set front left steer offset

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3; // FIXME Set front right drive motor ID
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4; // FIXME Set front right steer motor ID
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10; // FIXME Set front right steer encoder ID
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(
    340.83
  ); // FIXME Measure and set front right steer offset

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5; // FIXME Set back left drive motor ID
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; // FIXME Set back left steer motor ID
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11; // FIXME Set back left steer encoder ID
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(
    90
  ); // FIXME Measure and set back left steer offset

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7; // FIXME Set back right drive motor ID
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8; // FIXME Set back right steer motor ID
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12; // FIXME Set back right steer encoder ID
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(
    225
  ); // FIXME Measure and set back right steer offset

  public static final double SHOOTING_Kp = .25; //.01
  public static final double SHOOTING_Ki = 0; // .001
  public static final double SHOOTING_Kd = 0; // 70.5
  public static final double SHOOTING_Kf = 0.0639; //
  public static final double SHOOTING_Kizone = 0; // 50
  public static final double SHOOTING_TARGET_RPM_HIGH = 8000;
  public static final double SHOOTING_TARGET_RPM_LOW = 5000;
  public static final double SHOOTING_UNITS_PER_REV = 820;

  public static final double UPPER_BALL_SENSOR_THRESHOLD = 1000;
  public static final double LOWER_BALL_SENSOR_THRESHOLD = 1000;
  public static final double CLIMBER_HIGH_TICKS = 270000; //273141

  public static final double HUB_AIM_ANGLE_ERROR = 0.5; // The angle that the "TurnToHub" command will approximate
  public static final double HUB_CENTER_X = 8.22;
  public static final double HUB_CENTER_Y = 4.11;

  public static final class swerve {

    //speed 0-1
    public static final double TELEOPSPEED = 1;

    // ORDER: FL FR BL BR
    //6380 first num
    // 3.8
    // public static final double MAX_VEL_METERS = 3.8; // 3.7 worked with 2 inches of error at 8 max ang accel

    public static final double MAX_VEL_METERS =
      6380.0 /
      60.0 *
      SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
      SdsModuleConfigurations.MK4I_L2.getWheelDiameter() *
      Math.PI;

    // public static final double MAX_ANG_VEL_RAD = MAX_VEL_METERS
    //         / Math.hypot(Constants.dimensions.TRACKWIDTH / 2.0, Constants.dimensions.WHEELBASE / 2.0);

    //12

    public static final double MAX_VOLTAGE = 10; // 12

    public static final double MAX_ANG_ACCEL = 1 * Math.PI; //.5
    public static final boolean feildRelativeOn = true;
    public static final boolean brakeModeOn = true;
  }

  public static final class auto {

    /*
     * public static final Matrix<N3, N1> POSE_STD_DEV = new MatBuilder<>(Nat.N5(),
     * Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02), // State measurement standard
     * deviations. X, Y, theta. public static final Matrix<N3, N1> ENCODER_GYRO_DEV
     * = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local
     * measurement standard deviations. Left encoder, right encoder, gyro. public
     * static final Matrix<N3, N1> VISION_DEVIATION = new MatBuilder<>(Nat.N3(),
     * Nat.N1()).fill(0.1, 0.1, 0.01)); // Global measurement standard deviations.
     * X, Y, and theta.
     */

    public static final class follower {

      private static final double MAX_ANG_VEL_RAD_AUTO = .4 * Math.PI; //.25
      private static final double MAX_ANG_VEL_RAD_AUTO_3rdPath = .001 * Math.PI; //.25
      public static final TrapezoidProfile.Constraints ROT_PROFILE = new TrapezoidProfile.Constraints(
        MAX_ANG_VEL_RAD_AUTO,
        swerve.MAX_ANG_ACCEL
      );
      // x distance PID controller
      public static final PIDController X_PID_CONTROLLER = new PIDController(
        5,
        0,
        0
      ); // 5
      // y distance PID controller
      public static final PIDController Y_PID_CONTROLLER = new PIDController(
        5,
        0,
        0
      ); // 5, 0, .0 0.3, 0.4, 4
      // ROTATION (angle) PID controller
      public static final ProfiledPIDController ROT_PID_CONTROLLER = new ProfiledPIDController(
        -3,
        0,
        0, //.85 works
        ROT_PROFILE
      );
      // DRIVING DEFAULT IS 5
      public static final double LINEAR_VELOCITY_DEFAULT = 5; // no change
      // MUST SET KINEMATICS, see documentation
      public static final TrajectoryConfig T_CONFIG = new TrajectoryConfig(
        LINEAR_VELOCITY_DEFAULT,
        MAX_ANG_VEL_RAD_AUTO
      );
      public static final TrajectoryConfig T_CONFIG_3rdPath = new TrajectoryConfig(
        LINEAR_VELOCITY_DEFAULT,
        MAX_ANG_VEL_RAD_AUTO_3rdPath
      );
    }

    public static final class startingPos {

      public static final Pose2d DEFAULT_POS = new Pose2d();
    }
  }
}
