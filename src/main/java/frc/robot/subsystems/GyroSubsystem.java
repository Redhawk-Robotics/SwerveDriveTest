// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.modules.PiegeonModule;

public class GyroSubsystem extends SubsystemBase {

  private PiegeonModule pigeonModule;

  /** Creates a new Gyro. */
  public GyroSubsystem() {
    this.pigeonModule = PiegeonModule.getPigeonModule();
  }

  public void setYaw(double yaw) {
    this.pigeonModule.setYaw(yaw);
  }

  public double getYaw() {
    return this.pigeonModule.getYaw();
  }

  // Going to change, TODO TESTS
  public double getAnglePerpendicularToGroundDEG() {
    return this.pigeonModule.getYaw();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
