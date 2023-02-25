// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Ports;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.modules.PiegeonModule;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  
  private GyroSubsystem m_Gyro; 
  private SwerveSubsystem s_Swerve;


  public AutoBalance(SwerveSubsystem s_Swerve, GyroSubsystem m_Gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    this.m_Gyro = m_Gyro;
    addRequirements(s_Swerve, m_Gyro);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.m_Gyro.setYaw(0);
    SmartDashboard.getNumber("Yaw", this.m_Gyro.getYaw());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
