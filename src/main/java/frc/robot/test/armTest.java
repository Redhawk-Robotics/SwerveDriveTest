// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.Setting;
import frc.robot.lib.util.CANSparkMaxUtil;
import frc.robot.lib.util.CANSparkMaxUtil.Usage;

public class armTest extends SubsystemBase {
  /** Creates a new armTest. */
  private final CANSparkMax leftArmMotor, rightArmMotor;
  private final RelativeEncoder leftArmEncoder, rightArmEncoder;

  private final SparkMaxPIDController armAngleController;
  private double leftArmencoderValue;
  private double rightArmencoderValue;


  public armTest() {
    leftArmMotor = new CANSparkMax(Ports.arm.leftArm, MotorType.kBrushless);
    leftArmEncoder = leftArmMotor.getEncoder();  
    configLeftArmMotor();

    rightArmMotor = new CANSparkMax(Ports.arm.rightArm, MotorType.kBrushless);
    rightArmEncoder = rightArmMotor.getEncoder();  
    configRightArmMotor();
    //armAngleController = leftArmMotor.getPIDController();
    armAngleController = rightArmMotor.getPIDController();///TODO not sure if we need one or two PID controllers
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RightArm encoder value", getEncoderMetersLeft(leftArmencoderValue));
    SmartDashboard.putNumber("LeftArm encoder value", getEncoderMetersRight(rightArmencoderValue));
  }
  private void configLeftArmMotor() {
    leftArmMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(null, Usage.kPositionOnly);
    leftArmMotor.setSmartCurrentLimit(0);
    leftArmMotor.setIdleMode(null);
    leftArmEncoder.setVelocityConversionFactor(0);
    leftArmEncoder.setPositionConversionFactor(0);
    armAngleController.setP(0);
    armAngleController.setI(0);
    armAngleController.setD(0);
    armAngleController.setFF(0);
    leftArmMotor.enableVoltageCompensation(0.0);
    leftArmMotor.burnFlash();
    leftArmEncoder.setPosition(0.0);
    }
    private void configRightArmMotor() {
      rightArmMotor.restoreFactoryDefaults();
      CANSparkMaxUtil.setCANSparkMaxBusUsage(null, Usage.kPositionOnly);
      rightArmMotor.setSmartCurrentLimit(0);
      rightArmMotor.setIdleMode(null);
      rightArmEncoder.setVelocityConversionFactor(0);
      rightArmEncoder.setPositionConversionFactor(0);
      armAngleController.setP(0);
      armAngleController.setI(0);
      armAngleController.setD(0);
      armAngleController.setFF(0);
      rightArmMotor.enableVoltageCompensation(0.0);
      rightArmMotor.burnFlash();
      rightArmEncoder.setPosition(0.0);
      }
    public void setMotor(double speed) {
      leftArmMotor.set(speed);
      rightArmMotor.set(speed);
  }

  public double getEncoderMetersLeft(double positionLeft) {
    positionLeft = leftArmEncoder.getPosition() * Setting.ArmSetting.kEncoderTick2Meter;
    return positionLeft;
  }
  public double getEncoderMetersRight(double positionRight){
    positionRight = rightArmEncoder.getPosition() * Setting.ArmSetting.kEncoderTick2Meter;
    return positionRight;
  }
  
}
