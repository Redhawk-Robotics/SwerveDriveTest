// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.modules.PiegeonModule;

public class GyroSubsystem extends SubsystemBase {

  private PiegeonModule piegeonModule;

  /** Creates a new Gyro. */
  public GyroSubsystem() {
    this.piegeonModule = PiegeonModule.getPigeonModule();
  }

  public void setYaw(double yaw) {
    this.piegeonModule.setYaw(yaw);
  }

  public double getYaw() {
    return this.piegeonModule.getYaw();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
