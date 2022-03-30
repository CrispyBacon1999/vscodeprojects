// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class AutoPickUpBall extends CommandBase {

  /** Creates a new PickUpBall. */

  private Intake m_intake;
  private Magazine m_magazine;
  private Shooter m_shooter;
  private boolean commandFinished;
  private double m_shooteSpeed;
  private double runTimeCommand;
  private long endTimer = 0;
  private double value = 0;
  private double m_setHoodAngle;
  private final Timer timer = new Timer();

  public AutoPickUpBall(
    Intake intake,
    Magazine magazine,
    Shooter shooter,
    double shooteSpeed,
    double setHoodAngle
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_magazine = magazine;
    m_shooter = shooter;
    m_shooteSpeed = shooteSpeed;
    m_setHoodAngle = setHoodAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    commandFinished = false;
    m_intake.moveSolenoid(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("Ball Intake Running");
    // m_intake.moveIntake(-.4);
    // m_magazine.runLowerMag(0.7);
    // m_magazine.runUpperMag(-.7);
    // high no ball is ~250, with ball ~1100/
    m_shooter.customShootHigh(m_shooteSpeed);
    double currentPosition = m_shooter.getHoodPosition();
    double errorDis = currentPosition - m_setHoodAngle;
    // System.out.println("Location is : "+m_shooter.getHoodPosition());
    m_shooter.moveHood(errorDis * -.03);
    if (timer.get() > 10) {
      m_intake.moveIntake(0);
      m_magazine.runLowerMag(0);
      m_magazine.runUpperMag(0);
      m_intake.moveSolenoid(false);
      commandFinished = true;
    }
    if (
      m_magazine.getUpperBallSensor() > Constants.UPPER_BALL_SENSOR_THRESHOLD
    ) {
      if (
        m_magazine.getLowerBallSensor() < Constants.UPPER_BALL_SENSOR_THRESHOLD
      ) {
        // if (value == 0) {
        System.out.println("full");
        // value = 1;
        m_intake.moveIntake(0);
        m_magazine.runLowerMag(0);
        m_magazine.runUpperMag(0);
        // }

      } else {
        if (value == 0) {
          System.out.println("High ball in");
          value = 1;
          m_magazine.runUpperMag(0);
        }
        m_intake.moveIntake(-.8);
        m_magazine.runLowerMag(0.4);
      }
    } else {
      System.out.println("Empty");

      value = 0;
      m_intake.moveIntake(-.8);
      m_magazine.runLowerMag(0.4);
      m_magazine.runUpperMag(-.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.moveIntake(0);
    m_magazine.runLowerMag(0);
    m_magazine.runUpperMag(0);
    m_intake.moveSolenoid(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return commandFinished;
  }
}
