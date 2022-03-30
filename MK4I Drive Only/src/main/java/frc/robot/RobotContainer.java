package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants.auto.follower;
import frc.robot.commands.*;
import frc.robot.commands.Auto.FiveBallRight;
import frc.robot.commands.Auto.RunBasicTrajectory;
import frc.robot.commands.Auto.StraightPath;
import frc.robot.commands.Auto.ThreeBallRight;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import java.time.Instant;
import java.util.List;

public class RobotContainer {

  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final Magazine m_magazine = new Magazine();
  private final Climber m_climber = new Climber();
  private final Limelight m_limelight = new Limelight();
  private final XboxController m_controller1 = new XboxController(0);
  private final XboxController m_controller2 = new XboxController(1);
  private final XboxController m_testcontroller = new XboxController(2);
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final Compressor m_compressor = new Compressor(
      PneumaticsModuleType.REVPH);
  private double visionCorrection;
  private ThreeBallRight threeBallRight;
  private FiveBallRight fiveBallRight;
  private StraightPath straightPath;

  public RobotContainer() {
    threeBallRight = new ThreeBallRight(
        m_shooter,
        m_drivetrainSubsystem,
        m_intake,
        m_magazine);
    fiveBallRight = new FiveBallRight(m_shooter, m_drivetrainSubsystem, m_intake, m_magazine);
    straightPath = new StraightPath(m_shooter, m_drivetrainSubsystem);

    m_chooser.setDefaultOption("3 Ball Right", threeBallRight);
    m_chooser.addOption("5 Ball Right", fiveBallRight);
    m_chooser.addOption("Straight", straightPath);
    SmartDashboard.putData(m_chooser);

    m_drivetrainSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller1.getLeftY()) *
                DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller1.getLeftX()) *
                DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(-m_controller1.getRightX() + m_limelight.turnToTarget()) *
                DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    m_compressor.enableAnalog(115, 120);
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // *******************************
    // PRIMARY DRIVER: M_CONTROLLER1
    // *******************************
    // Zero Gyroscope
    new Button(m_controller1::getAButton)
        .whenPressed(m_drivetrainSubsystem::zeroGyroscope);
    // Intake Motors using sensors
    new Button(m_controller1::getRightBumper)
        .whenHeld(new PickUpBall(m_intake, m_magazine));
    // Intake down
    new Button(m_controller1::getRightBumper)
        .whenPressed(() -> m_intake.moveSolenoid(true))
        .whenReleased(() -> m_intake.moveSolenoid(false));
    // Reverse mag and shooter
    new Button(m_controller1::getLeftBumper)
        .whenActive(
            () -> {
              m_magazine.runLowerMag(-1);
              m_magazine.runUpperMag(.3);
              m_shooter.reverseShooter();
            })
        .whenInactive(
            () -> {
              m_magazine.runLowerMag(0);
              m_magazine.runUpperMag(0);
              m_shooter.stop();
            });
    // turn to target
    new Button(m_controller1::getYButton)
        .whenHeld(new TurnToTarget(m_limelight));

    // Passive hub tracking
    new Button(m_controller1::getXButton).whenPressed(new TurnToHub(m_drivetrainSubsystem,
        () -> -modifyAxis(m_controller1.getLeftY()) *
            DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_controller1.getLeftX()) *
            DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND));

    // *********************************
    // SECONDARY DRIVER: M_CONTROLLER2
    // *********************************
    // run hood to limit switch and reset encoder
    new Button(m_controller2::getAButton)
        .whenHeld(new CalibrateHood(m_shooter));
    // Adjust hood, rpm, and shoot ball from tarmac
    new Button(m_controller2::getRightBumper)
        .whenHeld(new ShootCustom(m_shooter, m_magazine, 11500, -3));
    // Pick up ball without intake
    new Button(m_controller2::getXButton)
        .whenHeld(new PickUpBallNoIntake(m_magazine));
    // Adjust hood, rpm, and shoot ball in low goal
    new Button(m_controller2::getYButton)
        .whenHeld(new ShootCustom(m_shooter, m_magazine, 6000, -5));
    // Adjust hood, rpm, and shoot ball from close safe zone
    new Button(m_controller2::getBButton)
        .whenHeld(new ShootCustom(m_shooter, m_magazine, 13000, -6));
    // force magazine up
    new Button(m_controller2::getLeftBumper)
        .whenActive(
            () -> {
              m_magazine.runLowerMag(.5);
              m_magazine.runUpperMag(-.5);
            })
        .whenInactive(
            () -> {
              m_magazine.runLowerMag(0);
              m_magazine.runUpperMag(0);
            });

    // *********************************
    // CLIMBER DRIVER: M_TESTCONTROLLER
    // *********************************
    // Retract arms
    new Button(m_testcontroller::getAButton)
        .whileHeld(() -> m_climber.runClimber(.6))
        .whenReleased(() -> m_climber.runClimber(0));
    // Extend arms
    new Button(m_testcontroller::getYButton)
        .whenHeld(new ClimbArmUp(m_climber));
    // Stop Shooter
    new Button(m_testcontroller::getYButton)
        .whenPressed(() -> m_shooter.stop());
    // Pneumatics out
    new Button(m_testcontroller::getXButton)
        .whenPressed(() -> m_climber.moveSolenoid(true));
    // Pneumatics in
    new Button(m_testcontroller::getBButton)
        .whenPressed(() -> m_climber.moveSolenoid(false));
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
