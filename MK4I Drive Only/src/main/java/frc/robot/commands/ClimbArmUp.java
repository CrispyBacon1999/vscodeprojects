// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import java.sql.Time;

public class ClimbArmUp extends CommandBase {

  private final Pigeon2 m_pigeon = new Pigeon2(13);
  private Timer timer = new Timer();
  private Climber m_climber;
  boolean armsUp = false;

  /** Creates a new Climb. */
  public ClimbArmUp(Climber climber) {
    m_climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //260000
    //274000
    //   if (m_climber.getLeftEncoderTicks() < -275000) {
    //     m_climber.runClimber(0);
    //   }
    // else {
    //   m_climber.runClimber(-.5);
    // }
    if (timer.get() < .5) {
      m_climber.runClimber(-.5);
    } else {
      m_climber.runClimber(-1);
    }
    //System.out.println(m_climber.getLeftEncoderTicks());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.runClimber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return armsUp;
    return false;
  }
}
