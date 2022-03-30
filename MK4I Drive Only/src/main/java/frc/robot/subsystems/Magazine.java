// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Magazine extends SubsystemBase {

  CANSparkMax lowerMagazine = new CANSparkMax(
    16,
    com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
  );
  CANSparkMax upperMagazine = new CANSparkMax(
    17,
    com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
  );
  AnalogInput lowerBallSensor = new AnalogInput(1);
  AnalogInput upperBallSensor = new AnalogInput(0);

  /** Creates a new Magazine. */
  public Magazine() {
    lowerMagazine.setIdleMode(IdleMode.kBrake);
    upperMagazine.setIdleMode(IdleMode.kBrake);
    SmartDashboard.putBoolean("UPPER", ballInUpper());
    SmartDashboard.putBoolean("LOWER", ballInLower());
  }

  public void runLowerMag(double speed) {
    lowerMagazine.set(speed);
  }

  public void runUpperMag(double speed) {
    upperMagazine.set(speed);
  }

  public boolean ballInUpper() {
    return upperBallSensor.getValue() > Constants.UPPER_BALL_SENSOR_THRESHOLD;
  }

  public boolean ballInLower() {
    return lowerBallSensor.getValue() < Constants.LOWER_BALL_SENSOR_THRESHOLD;
  }

  public double getLowerBallSensor() {
    return lowerBallSensor.getValue();
  }

  public double getUpperBallSensor() {
    //System.out.println("UPPER BALL SENSOR: " + upperBallSensor.getValue());
    return upperBallSensor.getValue();
  }

  public void stop() {
    lowerMagazine.set(0);
    upperMagazine.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("UPPER", ballInUpper());
    SmartDashboard.putBoolean("LOWER", ballInLower());
  }
}
