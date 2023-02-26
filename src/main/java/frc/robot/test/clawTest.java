// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.Setting;
import frc.robot.lib.util.CANSparkMaxUtil;
import frc.robot.lib.util.CANSparkMaxUtil.Usage;

public class clawTest extends SubsystemBase {

  private final CANSparkMax leftNeo550, rightNeo550;
  private final RelativeEncoder leftEncoder, righEncoder;
  private final SparkMaxPIDController leftPIDController, rightPIDController; 

  /** Creates a new clawTest. */
  public clawTest() {
    leftNeo550 = new CANSparkMax(Ports.claw.leftClaw, MotorType.kBrushless);
    rightNeo550 = new CANSparkMax(Ports.claw.rightClaw, MotorType.kBrushless);
    leftEncoder = leftNeo550.getEncoder();
    righEncoder = rightNeo550.getEncoder();
    leftPIDController = leftNeo550.getPIDController();
    rightPIDController = rightNeo550.getPIDController();

    configClawMotor(leftNeo550, leftEncoder, leftPIDController, Setting.ClawSetting.leftClawMotorInvert);
    configClawMotor(rightNeo550, righEncoder, rightPIDController, Setting.ClawSetting.rightClawMotorInvert);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runMotors() {
    leftNeo550.set(.5);
    rightNeo550.set(.5);
  }

  public void stopMotors() {
    leftNeo550.set(0);
    rightNeo550.set(0);
  }

  public void configClawMotor(CANSparkMax clawMotor, RelativeEncoder clawEncoder, SparkMaxPIDController clawController, boolean invert) {
    clawMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(clawMotor, Usage.kAll);
    clawMotor.setSmartCurrentLimit(Setting.ClawSetting.clawContinousCurrentLimit);
    clawMotor.setIdleMode(Setting.ClawSetting.clawNeutralMode);
    clawMotor.setInverted(invert);
    clawEncoder.setVelocityConversionFactor(Setting.ClawSetting.clawConversionVelocityFactor);
    clawEncoder.setPositionConversionFactor(Setting.ClawSetting.clawConversionPositionFactor);
    clawController.setP(Setting.ClawSetting.clawP);
    clawController.setI(Setting.ClawSetting.clawI);
    clawController.setD(Setting.ClawSetting.clawD);
    clawController.setFF(Setting.ClawSetting.clawFF);
    clawMotor.enableVoltageCompensation(Setting.ClawSetting.maxVoltage);
    clawMotor.burnFlash();
  }
}
