// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  CANSparkMax intake = new CANSparkMax(
    18,
    com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
  );
  Solenoid intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);

  /** Creates a new Intake. */
  public Intake() {
    intake.setIdleMode(IdleMode.kBrake);
  }

  public void moveIntake(double speed) {
    intake.set(speed);
  }

  public void moveSolenoid(boolean solenoidPose) {
    intakeSolenoid.set(solenoidPose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
