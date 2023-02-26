// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
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
  private double leftArmEncoderValue;
  private double rightArmEncoderValue;


  public armTest() {
    leftArmMotor = new CANSparkMax(Ports.arm.leftArm, MotorType.kBrushless);
    leftArmEncoder = leftArmMotor.getEncoder();  

    rightArmMotor = new CANSparkMax(Ports.arm.rightArm, MotorType.kBrushless);
    rightArmEncoder = rightArmMotor.getEncoder();  

    //armAngleController = leftArmMotor.getPIDController();
    armAngleController = rightArmMotor.getPIDController();//TODO not sure if we need one or two PID controllers

    configArmMotor(leftArmMotor,leftArmEncoder,armAngleController,Setting.ArmSetting.leftArmMotorInvert);

    configArmMotor(rightArmMotor,rightArmEncoder,armAngleController,Setting.ArmSetting.rightArmMotorInvert);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RightArm Encoder Value", getEncoderMetersLeft(leftArmEncoderValue));
    SmartDashboard.putNumber("LeftArm Encoder Value", getEncoderMetersRight(rightArmEncoderValue));
  }
  private void configArmMotor(CANSparkMax ArmMotor, RelativeEncoder ArmEncoder, SparkMaxPIDController armAngleController, boolean Invert) {
    ArmMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(ArmMotor, Usage.kPositionOnly);
    ArmMotor.setSmartCurrentLimit(Setting.ArmSetting.armContinousCurrentLimit);
    ArmMotor.setInverted(Invert);
    ArmMotor.setIdleMode(Setting.ArmSetting.armNeutralMode);
    ArmEncoder.setPositionConversionFactor(Setting.ArmSetting.armConversionFactor);
    armAngleController.setP(Setting.ArmSetting.armP);
    armAngleController.setI(Setting.ArmSetting.armI);
    armAngleController.setD(Setting.ArmSetting.armD);
    armAngleController.setFF(Setting.ArmSetting.armFF);
    ArmMotor.enableVoltageCompensation(Setting.ArmSetting.maxVoltage);
    ArmMotor.burnFlash();
    Timer.delay(1);
    //resetToAbsolute();//FIXME if we are adding a canCODER to the shaft of the arm
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
