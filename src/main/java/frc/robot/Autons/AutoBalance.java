// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Ports;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  
  private Pigeon2 m_Pigeon = new Pigeon2(Ports.Gyro.DRIVETRAIN_PIGEON_ID);
  private SwerveSubsystem s_Swerve;


  public AutoBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(s_Swerve);  
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SmartDashboard.getNumber("Yaw",m_Pigeon.getYaw());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
