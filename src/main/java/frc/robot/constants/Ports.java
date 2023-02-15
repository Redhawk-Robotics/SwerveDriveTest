// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @Filename:Ports.java
 * @Purpose:This file contains the different ports of motors, and sensors
 * @Version:1.0
 * @Author: TEAM 8739 RedhawkRobotics
 * @Date: 02/07/23
 */

package frc.robot.constants;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.SwerveModuleConstants;

public interface Ports {
/* Swerve Voltage Compensation */
public static final double voltageComp = 6.0;

public static final double stickDeadband = 0.1;

public interface Gamepad{
    public static final int DRIVER = 0;
    public static final int OPERATOR = 1;
    public static final int DEBUGGER = 2;
}
public interface Gyro{
    public static final int DRIVETRAIN_PIGEON_ID = 39;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
}

/* Drivetrain Constants */
public static final double trackWidth = Units.inchesToMeters(28);
public static final double wheelBase = Units.inchesToMeters(28);
public static final double wheelDiameter = Units.inchesToMeters(4.0);
public static final double wheelCircumference = wheelDiameter * Math.PI;

//Module Constants
public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

/* Neutral Modes */
public static final IdleMode angleNeutralMode = IdleMode.kBrake;
public static final IdleMode driveNeutralMode = IdleMode.kBrake;

/* Motor Inverts */
public static final boolean driveInvert = false;
public static final boolean angleInvert = false;

/* Angle Encoder Invert */
public static final boolean canCoderInvert = false;

//SwerveKinematics
public static final SwerveDriveKinematics swerveKinematics =
new SwerveDriveKinematics(
    new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
    new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
    new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
    new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

//FRONT_LEFT_MODULE
public static final class FRONT_LEFT_MODULE{
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; // FIXME Set front left module drive motor ID 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 11; // FIXME Set front left steer encoder ID 
    public static final Rotation2d FRONT_LEFT_MODULE_STEER_OFFSET = Rotation2d.fromDegrees(0.0); // FIXME Measure and set front left steer offset
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(FRONT_LEFT_MODULE_DRIVE_MOTOR,
    FRONT_LEFT_MODULE_STEER_MOTOR, FRONT_LEFT_MODULE_STEER_ENCODER, FRONT_LEFT_MODULE_STEER_OFFSET);
}
            
//FRONT_RIGHT_MODULE
public static final class FRONT_RIGHT_MODULE{
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22; // FIXME Set front right steer encoder ID
    public static final Rotation2d FRONT_RIGHT_MODULE_STEER_OFFSET = Rotation2d.fromDegrees(0.0); // FIXME Measure and set front right steer offset
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(FRONT_RIGHT_MODULE_DRIVE_MOTOR,
    FRONT_RIGHT_MODULE_STEER_MOTOR, FRONT_RIGHT_MODULE_STEER_ENCODER, FRONT_RIGHT_MODULE_STEER_OFFSET);
}

//BACK_LEFT_MODULE
public static final class BACK_LEFT_MODULE{
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5; // FIXME Set back left drive motor ID 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; // FIXME Set back left steer motor ID 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 33; // FIXME Set back left steer encoder ID 
    public static final Rotation2d BACK_LEFT_MODULE_STEER_OFFSET = Rotation2d.fromDegrees(0.0); // FIXME Measure and set back left steer offset
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(BACK_LEFT_MODULE_DRIVE_MOTOR,
    BACK_LEFT_MODULE_STEER_MOTOR, BACK_LEFT_MODULE_STEER_ENCODER, BACK_LEFT_MODULE_STEER_OFFSET);
}
//BACK_RIGHT_MODULE
public static final class BACK_RIGHT_MODULE{
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8; // FIXME Set back right steer motor ID 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 44; // FIXME Set back right steer encoder ID 
    public static final Rotation2d BACK_RIGHT_MODULE_STEER_OFFSET = Rotation2d.fromDegrees(0.0); // FIXME Measure and set back right steer offset
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(BACK_RIGHT_MODULE_DRIVE_MOTOR,
    BACK_RIGHT_MODULE_STEER_MOTOR, BACK_RIGHT_MODULE_STEER_ENCODER, BACK_RIGHT_MODULE_STEER_OFFSET);
    }

/* Swerve Profiling Values */
public static final double maxSpeed = 0; // meters per second //FIXME
public static final double maxAngularVelocity = 0;//FIXME


/* Swerve Current Limiting */
public static final int angleContinuousCurrentLimit = 0;//FIXME
public static final int driveContinuousCurrentLimit = 0;//FIXME

 /* Angle Motor PID Values */
 public static final double angleKP = 0.0;//FIXME
 public static final double angleKI = 0.0;//FIXME
 public static final double angleKD = 0.0;//FIXME
 public static final double angleKFF = 0.0;//FIXME

 /* Drive Motor PID Values */
 public static final double driveKP = 0.0;//FIXME
 public static final double driveKI = 0.0;//FIXME
 public static final double driveKD = 0.0;//FIXME
 public static final double driveKFF = 0.0;//FIXME

 /* Drive Motor Characterization Values */
 public static final double driveKS = 0.0;//FIXME
 public static final double driveKV = 0.0;//FIXME
 public static final double driveKA = 0.0;//FIXME


 public static final class AutoConstants {

 }
}
