// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimbArmUp6in extends CommandBase {

  private Climber m_climber;
  boolean armsUp = false;

  /** Creates a new Climb. */
  public ClimbArmUp6in(Climber climber) {
    m_climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double right = m_climber.getRightEncoderTicks();
    double left = m_climber.getLeftEncoderTicks();
    if (right > 30000 && left < -30000) {
      //done
      m_climber.runClimber(0);
      armsUp = true;
    } else {
      m_climber.runClimber(-.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.runClimber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armsUp;
  }
}
