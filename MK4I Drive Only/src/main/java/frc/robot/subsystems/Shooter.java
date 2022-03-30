// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  TalonFX leftShooter = new TalonFX(19);
  TalonFX rightShooter = new TalonFX(20);
  CANSparkMax hood = new CANSparkMax(
    21,
    com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
  );
  RelativeEncoder encoder = hood.getEncoder();
  SparkMaxLimitSwitch backLimitSwitch = hood.getForwardLimitSwitch(
    SparkMaxLimitSwitch.Type.kNormallyOpen
  ); //back
  SparkMaxLimitSwitch forwardLimitSwitch = hood.getReverseLimitSwitch(
    SparkMaxLimitSwitch.Type.kNormallyOpen
  ); //forward

  // Magazine m_magazine = new Magazine();

  /** Creates a new Shooter. */
  public Shooter() {
    leftShooter.setNeutralMode(NeutralMode.Brake);
    rightShooter.setNeutralMode(NeutralMode.Brake);
    customShootHigh(0);
    hood.setIdleMode(IdleMode.kBrake);
    rightShooter.setInverted(true);
    rightShooter.follow(leftShooter, FollowerType.AuxOutput1);

    leftShooter.config_kP(0, 0, 30);
    leftShooter.config_kI(0, 0, 30);
    leftShooter.config_kD(0, 0, 30);
    leftShooter.config_kF(0, 0, 30);
    leftShooter.config_IntegralZone(0, 50);
    rightShooter.config_kP(0, 0, 30);
    rightShooter.config_kI(0, 0, 30);
    rightShooter.config_kD(0, 0, 30);
    rightShooter.config_kF(0, 0, 30);
    rightShooter.config_IntegralZone(0, 50);
  }

  public void shoot() {
    leftShooter.set(ControlMode.PercentOutput, 0.75);
    rightShooter.set(ControlMode.PercentOutput, -0.75);
  }

  public void shootHigh() {
    leftShooter.config_kP(0, Constants.SHOOTING_Kp, 30);
    leftShooter.config_kI(0, Constants.SHOOTING_Ki, 30);
    leftShooter.config_kD(0, Constants.SHOOTING_Kd, 30);
    leftShooter.config_kF(0, Constants.SHOOTING_Kf, 30);
    leftShooter.config_IntegralZone(0, 50);
    rightShooter.config_kP(0, Constants.SHOOTING_Kp, 30);
    rightShooter.config_kI(0, Constants.SHOOTING_Ki, 30);
    rightShooter.config_kD(0, Constants.SHOOTING_Kd, 30);
    rightShooter.config_kF(0, Constants.SHOOTING_Kf, 30);
    rightShooter.config_IntegralZone(0, 50);
    leftShooter.set(
      ControlMode.Velocity,
      Constants.SHOOTING_UNITS_PER_REV *
      Constants.SHOOTING_TARGET_RPM_HIGH /
      600
    );
    rightShooter.set(
      ControlMode.Velocity,
      Constants.SHOOTING_UNITS_PER_REV *
      Constants.SHOOTING_TARGET_RPM_HIGH /
      600
    );
  }

  public void customShootHigh(double velocity) {
    leftShooter.config_kP(0, Constants.SHOOTING_Kp, 30);
    leftShooter.config_kI(0, Constants.SHOOTING_Ki, 30);
    leftShooter.config_kD(0, Constants.SHOOTING_Kd, 30);
    leftShooter.config_kF(0, Constants.SHOOTING_Kf, 30);
    leftShooter.config_IntegralZone(0, 50);
    rightShooter.config_kP(0, Constants.SHOOTING_Kp, 30);
    rightShooter.config_kI(0, Constants.SHOOTING_Ki, 30);
    rightShooter.config_kD(0, Constants.SHOOTING_Kd, 30);
    rightShooter.config_kF(0, Constants.SHOOTING_Kf, 30);
    rightShooter.config_IntegralZone(0, 50);
    leftShooter.set(ControlMode.Velocity, velocity);
    rightShooter.set(ControlMode.Velocity, velocity);
  }

  public void shootLow() {
    leftShooter.config_kP(0, Constants.SHOOTING_Kp, 30);
    leftShooter.config_kI(0, Constants.SHOOTING_Ki, 30);
    leftShooter.config_kD(0, Constants.SHOOTING_Kd, 30);
    leftShooter.config_kF(0, Constants.SHOOTING_Kf, 30);
    leftShooter.config_IntegralZone(0, 50);
    rightShooter.config_kP(0, Constants.SHOOTING_Kp, 30);
    rightShooter.config_kI(0, Constants.SHOOTING_Ki, 30);
    rightShooter.config_kD(0, Constants.SHOOTING_Kd, 30);
    rightShooter.config_kF(0, Constants.SHOOTING_Kf, 30);
    rightShooter.config_IntegralZone(0, 50);

    leftShooter.set(
      ControlMode.Velocity,
      Constants.SHOOTING_UNITS_PER_REV * Constants.SHOOTING_TARGET_RPM_LOW / 600
    );
    rightShooter.set(
      ControlMode.Velocity,
      Constants.SHOOTING_UNITS_PER_REV * Constants.SHOOTING_TARGET_RPM_LOW / 600
    );
  }

  public double getShooterError() {
    return leftShooter.getClosedLoopError();
  }

  public void stop() {
    customShootHigh(0);
    leftShooter.config_kP(0, 0, 30);
    leftShooter.config_kI(0, 0, 30);
    leftShooter.config_kD(0, 0, 30);
    leftShooter.config_kF(0, 0, 30);
    leftShooter.config_IntegralZone(0, 50);
    rightShooter.config_kP(0, 0, 30);
    rightShooter.config_kI(0, 0, 30);
    rightShooter.config_kD(0, 0, 30);
    rightShooter.config_kF(0, 0, 30);
    rightShooter.config_IntegralZone(0, 50);
    leftShooter.set(ControlMode.PercentOutput, 0.0);
    rightShooter.set(ControlMode.PercentOutput, 0.0);
  }

  public boolean isBackLimitSwitchTriggered() {
    return backLimitSwitch.isPressed();
  }

  public boolean isForwardLimitSwitchTriggered() {
    return forwardLimitSwitch.isPressed();
  }

  public void moveHood(double power) {
    hood.set(power);
  }

  public double relativeLeftEncoderTicks(double ticks) {
    return 0;
  }

  public void zeroEncoderOfHood() {
    hood.getEncoder().setPosition(0);
  }

  public void reverseShooter() {
    leftShooter.set(ControlMode.PercentOutput, -.5);
    rightShooter.set(ControlMode.PercentOutput, -.5);
  }

  public double motorOutput() {
    return leftShooter.getMotorOutputPercent();
  }

  public double leftSpeed() {
    return leftShooter.getSelectedSensorVelocity();
  }

  public double getHoodPosition() {
    return encoder.getPosition();
  }

  @Override
  public void periodic() {}
}
