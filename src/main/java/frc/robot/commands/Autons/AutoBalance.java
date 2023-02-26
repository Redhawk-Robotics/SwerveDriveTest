// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Setting;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */

  private GyroSubsystem m_Gyro;
  private SwerveSubsystem s_Swerve;
  private Timer time;
  private double prevTime = 0;
  private double setpoint = 0; // 0 degress means the robot is balanced
  private double prevError = 0;
  private double sumOfError = 0;
  private final double chargePadLengthMETERS = 4. / 3.281;
  private final double chargePadTiltDEG = 16.5;
  private boolean isFinishedBool = false;
  private double Kp = 1;
  private double Ki = 0;
  private double Kd = 0;

  public AutoBalance(SwerveSubsystem s_Swerve, GyroSubsystem m_Gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    this.m_Gyro = m_Gyro;
    addRequirements(s_Swerve, m_Gyro);

    time = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    System.out.println(this.m_Gyro.getPitch());
    SmartDashboard.getNumber("Pitch", this.m_Gyro.getPitch());
    isFinishedBool = false;
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currTime = time.get();
    double deltaTime = currTime - prevTime;
    System.out.println("Deltatime: " + deltaTime);

    double error = setpoint - this.m_Gyro.getAnglePerpendicularToGroundDEG();
    System.out.println("Error: " + error);

    double output = (Kp * error) + (Ki * (sumOfError + (error * deltaTime))) + (Kd * ((error - prevError) / deltaTime));
    System.out.println("Output: " + output);

    // TODO fix MAX_VELOCITY_METERS_PER_SECOND (bros going too fast)
     // Assume 15* tilt means MAX transliation
    double strafeVal = (Setting.MAX_VELOCITY_METERS_PER_SECOND / chargePadTiltDEG) * output;
    System.out.println("strafeVal: " + strafeVal);

    // Might need to clamp the transilation values
    s_Swerve.drive(
        new Translation2d(0, strafeVal),
        0,
        true,
        true);

    prevTime = currTime;
    sumOfError += error;
    prevError = error;

    // test if we should stop?
    if (Math.abs(error) < 2) {
      // perhaps we should check if the error hasn't changed in the elapsed X time
      isFinishedBool = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinishedBool;
  }
}
