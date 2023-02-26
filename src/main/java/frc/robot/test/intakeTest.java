// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class intakeTest extends SubsystemBase {

  public final Compressor compressor;

  public final DoubleSolenoid leftSolenoid;
  public final DoubleSolenoid rightSolenoid;
  /** Creates a new intakeTest. */
  public intakeTest() {
    this.compressor = new Compressor(PneumaticsModuleType.REVPH);
    this.leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    this.rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    compressor.enableAnalog(0, 120);
    leftSolenoid.set(kReverse);
    rightSolenoid.set(kReverse);
  }

  public void intakeDown() {
      leftSolenoid.set(kForward);
      rightSolenoid.set(kForward);
    }
}
