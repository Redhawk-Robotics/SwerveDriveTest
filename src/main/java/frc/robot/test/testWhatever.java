// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Setting;

public class testWhatever extends SubsystemBase {
  /** Creates a new testWhatever. */
  private final CANSparkMax testMotor;
  private final RelativeEncoder testEncoder;
  
  private double EncoderValue;

public testWhatever() {
testMotor = new CANSparkMax(0, MotorType.kBrushless);
testEncoder = testMotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("testMotor Encoder Value", getEncoderMeters(EncoderValue));
  }
  public void setMotor(double speed) {
    testMotor.set(speed);
 } 
  public double getEncoderMeters(double positionLeft) {
    positionLeft = testEncoder.getPosition() * Setting.armSetting.kEncoderTick2Meter;
    return positionLeft;
  }
}
