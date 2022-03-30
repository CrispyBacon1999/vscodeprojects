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
import java.sql.Time;

public class AutoShootCommand extends CommandBase {

  /** Creates a new ShootHigh. */

  private Shooter m_shooter;
  private Magazine m_magazine;
  private double m_RPM;
  private Boolean commandFinished;
  double runTimeCommand = 0;
  private Intake m_intake;
  private final Timer timer = new Timer();

  public AutoShootCommand(Magazine magazine, Shooter shooter, double RPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_magazine = magazine;
    m_RPM = RPM;
    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    commandFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (
      m_magazine.getUpperBallSensor() > Constants.UPPER_BALL_SENSOR_THRESHOLD
    ) {
      if (
        m_shooter.leftSpeed() < m_RPM * 1.1 &&
        m_shooter.leftSpeed() > m_RPM * 0.9
      ) {
        //System.out.println("High Ball Shoting");
        m_magazine.runUpperMag(-.4);
      }
    } else {
      //System.out.println("Low Ball Shoting");
      m_magazine.runUpperMag(0);
      commandFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_magazine.runUpperMag(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return commandFinished;
  }
}
