// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Setting;

public class testWhatever extends SubsystemBase {
  /** Creates a new testWhatever. */
  private final CANSparkMax armMotorleft;
  private final CANSparkMax armMotorright;
  private final RelativeEncoder armEncoder;
  private final RelativeEncoder armEncoderright;

  //compressor
  private final CANSparkMax leftClaw;
  private final CANSparkMax rightClaw;

  private final CANSparkMax wrist;
  private final CANSparkMax extender;

  private final DoubleSolenoid clawOpen;


  private double value = 0.5;
  
  private double EncoderValue;
  private double stop = 0;

public testWhatever() {
armMotorleft = new CANSparkMax(16, MotorType.kBrushless);
armMotorright = new CANSparkMax(21, MotorType.kBrushless);

leftClaw = new CANSparkMax(12, MotorType.kBrushless);
rightClaw = new CANSparkMax(49, MotorType.kBrushless);

wrist = new CANSparkMax(24, MotorType.kBrushless);


extender = new CANSparkMax(22, MotorType.kBrushless);


clawOpen = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

extender.setInverted(true);

wrist.setInverted(false);

leftClaw.setInverted(false);
rightClaw.setInverted(true);;


armMotorleft.setInverted(false);
armMotorright.setInverted(true);

armEncoder = armMotorleft.getEncoder();
armEncoderright = armMotorright.getEncoder();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("testMotor Encoder Value", getEncoderMeters(EncoderValue));
  }
  //arm
  public void setMotor(double speed) {
    armMotorleft.set(speed);
    armMotorright.set(speed);
 } 
 //claw motors
 public void setMotorClaw(double speed) {
  rightClaw.set(speed);
  leftClaw.set(speed);
} 
  public double getEncoderMeters(double positionLeft) {
    positionLeft = armEncoder.getPosition() * Setting.armSetting.kEncoderTick2Meter;
    return positionLeft;
  }
  public double getEncoderMetersRight(double positionRight) {
    positionRight = armEncoder.getPosition() * Setting.armSetting.kEncoderTick2Meter;
    return positionRight;
  }

  //arm
  public void upGoArm() {
    armMotorleft.set(.3);
    armMotorright.set(.3);
  }

  public void downGoArm() {
    armMotorleft.set(-.3 / 2);
    armMotorright.set(-.3 / 2);
  }

  public void stopArm() {
    armMotorleft.set(stop);
    armMotorright.set(stop);
  }
  

  //Claw
  public void upGoClaw() {
    leftClaw.set(value);
    rightClaw.set(value);
  }  
  public void downGoClaw() {
    leftClaw.set(-value / 2);
    rightClaw.set(-value / 2);
  }

  public void stopClaw() {
    leftClaw.set(stop);
    rightClaw.set(stop);
  }

//Pneumatics with claw
  public void closeClaw(){
    clawOpen.set(Value.kReverse);
  }
  public void openClaw(){
    clawOpen.set(Value.kForward);
  }
  
  //Wrist
  public void upGoWrist() {
    wrist.set(value);
  }

  public void stopWrist() {
    wrist.set(stop);
  }
  public void downGoWrist() {
    wrist.set(-value / 2);
  }

  //Extender
  public void upExtender() {
    extender.set(value);
  }

  public void stopExtender() {
    extender.set(stop);
  }
  public void downExtender() {
    extender.set(-value / 2);
  }
}
