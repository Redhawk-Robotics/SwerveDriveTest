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
  private final CANSparkMax armMotorLeft;
  private final CANSparkMax armMotorRight;

  private final RelativeEncoder armEncoderLeft;
  private final RelativeEncoder armEncoderRight;

  //compressor
  private final CANSparkMax leftClaw;
  private final CANSparkMax rightClaw;


  private final CANSparkMax wrist;
  private final RelativeEncoder wristEncoder;

  private final CANSparkMax extender;
  private final RelativeEncoder extenderEncoder;

  private final DoubleSolenoid clawOpen;

  private double stop = 0;

  private double extenderSpeed = 1;
  private double extenderSpeedReverse = -1;

  private double wristSpeed = 0.25;
  private double wristSpeedReverse = -0.25;

  private double armSpeed = 0.1;
  private double armSpeedReverse = -0.1;

  private double clawSpeed = 0.5;
  private double clawSpeedReverse = -0.5;

  private double EncoderValue;

public testWhatever() {
armMotorLeft = new CANSparkMax(62, MotorType.kBrushless);
armMotorRight = new CANSparkMax(24, MotorType.kBrushless);

leftClaw = new CANSparkMax(16, MotorType.kBrushless);
rightClaw = new CANSparkMax(21, MotorType.kBrushless);

wrist = new CANSparkMax(27, MotorType.kBrushless);


extender = new CANSparkMax(49, MotorType.kBrushless);


clawOpen = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

extender.setInverted(true);

wrist.setInverted(false);

leftClaw.setInverted(false);
rightClaw.setInverted(true);;


armMotorLeft.setInverted(false);
armMotorRight.setInverted(true);

armEncoderLeft = armMotorLeft.getEncoder();
armEncoderRight = armMotorRight.getEncoder();

wristEncoder = wrist.getEncoder();
extenderEncoder = extender.getEncoder();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("testMotor Encoder Value", getEncoderMeters(EncoderValue));
  }
  //arm
  public void setMotor(double speed) {
    armMotorLeft.set(speed);
    armMotorRight.set(speed);
 } 
 //claw motors
 public void setMotorClaw(double speed) {
  rightClaw.set(speed);
  leftClaw.set(speed);
} 
  public double getEncoderMetersLeftArm(double positionLeft) {
    positionLeft = armEncoderLeft.getPosition() * Setting.armSetting.kEncoderTick2Meter;
    return positionLeft;
  }
  public double getEncoderMetersRightArm(double positionRight) {
    positionRight = armEncoderRight.getPosition() * Setting.armSetting.kEncoderTick2Meter;
    return positionRight;
  }

  public double getEncoderMetersExtender(double positonExtender){
    positonExtender = extenderEncoder.getPosition() * Setting.armSetting.kEncoderTick2Meter;
    return positonExtender;
  }

  public double getEncoderMetersWrist(double positonWrist){
    positonWrist = wristEncoder.getPosition() * Setting.armSetting.kEncoderTick2Meter;
    return positonWrist;
  }


  //arm
  public void upGoArm() {
    armMotorLeft.set(armSpeed);
    armMotorRight.set(armSpeed);
  }

  public void downGoArm() {
    armMotorLeft.set(armSpeedReverse);
    armMotorRight.set(armSpeedReverse);
  }

  public void stopArm() {
    armMotorLeft.set(stop);
    armMotorRight.set(stop);
  }
  

  //Claw
  public void upGoClaw() {
    leftClaw.set(clawSpeed);
    rightClaw.set(clawSpeed);
  }  
  public void downGoClaw() {
    leftClaw.set(clawSpeedReverse);
    rightClaw.set(clawSpeedReverse);
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
    wrist.set(wristSpeed);
  }

  public void stopWrist() {
    wrist.set(stop);
  }
  public void downGoWrist() {
    wrist.set(wristSpeedReverse);
  }

  //Extender
  public void upExtender() {
    extender.set(extenderSpeed);
  }

  public void stopExtender() {
    extender.set(stop);
  }
  public void downExtender() {
    extender.set(extenderSpeedReverse);
  }
}
