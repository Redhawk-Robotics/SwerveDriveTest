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

    configClawMotor(leftNeo550, leftEncoder, leftPIDController);
    configClawMotor(rightNeo550, righEncoder, rightPIDController);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void configClawMotor(CANSparkMax clawMotor, RelativeEncoder clawEncoder, SparkMaxPIDController clawController) {
    clawMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(null, Usage.kAll);
    clawMotor.setSmartCurrentLimit(0);
    clawMotor.setIdleMode(null);
    clawEncoder.setVelocityConversionFactor(0);
    clawEncoder.setPositionConversionFactor(0);
    clawController.setP(0);
    clawController.setI(0);
    clawController.setD(0);
    clawController.setFF(0);
    clawMotor.enableVoltageCompensation(0);
    clawMotor.burnFlash();
    clawEncoder.setPosition(0.0);
  }
}
