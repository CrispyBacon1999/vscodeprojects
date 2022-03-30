// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  /** Creates a new Limelight. */

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tshort = table.getEntry("tshort");
  NetworkTableEntry tlong = table.getEntry("tlong");

  double x = tx.getDouble(0.0);
  double y = ty.getDouble(0.0);
  double area = ta.getDouble(0.0);
  double valid = tv.getDouble(0.0);
  double shortside = tshort.getDouble(0.0);
  double longside = tlong.getDouble(0.0);
  double kP = .03;
  double output;
  Boolean targetingOn = false;

  public Limelight() {
    SmartDashboard.putBoolean("DISTANCE", tarmacDistance());
    SmartDashboard.putBoolean("ANGLE", alignGood());
  }

  public double getY() {
    return y;
  }

  public double getX() {
    return tx.getDouble(0.0);
  }

  public double ifValidTarget() {
    return valid;
  }

  public double getShortSide() {
    return shortside;
  }

  public double getLongSide() {
    return longside;
  }

  public boolean tarmacDistance() {
    if (ty.getDouble(0.0) > 6 && ty.getDouble(0.0) < 10) {
      return true;
    } else {
      return false;
    }
  }

  public boolean alignGood() {
    if (tx.getDouble(0.0) > -3 && tx.getDouble(0.0) < 3) {
      return true;
    } else {
      return false;
    }
  }

  public void targetingOnFunc() {
    targetingOn = true;
  }

  public void targetToggle(boolean onOff) {
    targetingOn = onOff;
  }

  public boolean isTargeting() {
    return targetingOn;
  }

  public double turnToTarget() {
    // double x = getX();
    // System.out.println(getX());
    if (targetingOn) {
      // System.out.println(getX());
      if (getX() > 1) {
        if (getX() > 5) {
          output = -.04 * getX();
        } else {
          output = -.02 * getX();
        }
      }
      if (getX() < -1) {
        if (getX() < -5) {
          output = -.04 * getX();
        } else {
          output = -.02 * getX();
        }
      }
      if (Math.abs(getX() * -.04) > .5) {
        output = Math.signum(getX()) * -.5;
      }
      return output;
    }
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("DISTANCE", tarmacDistance());
  }
}
