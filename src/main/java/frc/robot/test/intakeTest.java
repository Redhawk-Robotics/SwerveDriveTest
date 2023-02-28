// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Setting;
import frc.robot.subsystems.modules.CompressorModule;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class intakeTest extends SubsystemBase {

  public final CompressorModule compressor;
  public final DoubleSolenoid leftSolenoid, rightSolenoid;

  /** Creates a new intakeTest. */
  public intakeTest() {
    this.compressor = CompressorModule.getCompressorModule();
    this.leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Setting.intakePneumatics.leftForwardChan, Setting.intakePneumatics.leftReverseChan);
    this.rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Setting.intakePneumatics.leftForwardChan, Setting.intakePneumatics.leftForwardChan);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    compressor.enableAnalog(Setting.compressor.absoluteMinPressure, Setting.compressor.absoluteMaxPressure);
    SmartDashboard.putBoolean("Reached max pressure", compressor.isEnabled() ? true : false);
    SmartDashboard.putNumber("Pressure", compressor.getPressure());
    intakeUp();
  }

  public void intakeDown() {
    leftSolenoid.set(kForward);
    rightSolenoid.set(kForward);
  }

  public void intakeUp() {
    leftSolenoid.set(kReverse);
    rightSolenoid.set(kReverse);
  }
}
