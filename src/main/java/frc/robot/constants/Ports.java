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

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.lib.SwerveModuleConstants;

public interface Ports {

public static final double stickDeadband = 0.1;//may be 0.01

public static final class Gamepad{
    public static final int DRIVER = 0;
    public static final int OPERATOR = 1;
    public static final int DEBUGGER = 2;
}
public static final class Gyro{
    public static final int DRIVETRAIN_PIGEON_ID = 39;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
}
public static final class arm{
public static final int leftArm = 13;
public static final int rightArm = 14;
}
public static final class extend{
public static final int extend = 15;
}
public static final class claw{
public static final int leftClaw = 16;
public static final int rightClaw = 17;
}
//FRONT_LEFT_MODULE
public static final class FRONT_LEFT_MODULE0{
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; // FIXME Set front left module drive motor ID 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 11; // FIXME Set front left steer encoder ID 
    public static final Rotation2d FRONT_LEFT_MODULE_STEER_OFFSET = Rotation2d.fromDegrees(297.59765625); // FIXME Measure and set front left steer offset
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(FRONT_LEFT_MODULE_DRIVE_MOTOR,
    FRONT_LEFT_MODULE_STEER_MOTOR, FRONT_LEFT_MODULE_STEER_ENCODER, FRONT_LEFT_MODULE_STEER_OFFSET);
}
            
//FRONT_RIGHT_MODULE
public static final class FRONT_RIGHT_MODULE1{
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22; // FIXME Set front right steer encoder ID
    public static final Rotation2d FRONT_RIGHT_MODULE_STEER_OFFSET = Rotation2d.fromDegrees(247.5); // FIXME Measure and set front right steer offset
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(FRONT_RIGHT_MODULE_DRIVE_MOTOR,
    FRONT_RIGHT_MODULE_STEER_MOTOR, FRONT_RIGHT_MODULE_STEER_ENCODER, FRONT_RIGHT_MODULE_STEER_OFFSET);
}

//BACK_LEFT_MODULE
public static final class BACK_LEFT_MODULE2{
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5; // FIXME Set back left drive motor ID 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; // FIXME Set back left steer motor ID 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 33; // FIXME Set back left steer encoder ID 
    public static final Rotation2d BACK_LEFT_MODULE_STEER_OFFSET = Rotation2d.fromDegrees(229.658203125); // FIXME Measure and set back left steer offset
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(BACK_LEFT_MODULE_DRIVE_MOTOR,
    BACK_LEFT_MODULE_STEER_MOTOR, BACK_LEFT_MODULE_STEER_ENCODER, BACK_LEFT_MODULE_STEER_OFFSET);
}
//BACK_RIGHT_MODULE
public static final class BACK_RIGHT_MODULE3{
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8; // FIXME Set back right steer motor ID 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 44; // FIXME Set back right steer encoder ID 
    public static final Rotation2d BACK_RIGHT_MODULE_STEER_OFFSET = Rotation2d.fromDegrees(216.73828125); // FIXME Measure and set back right steer offset
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(BACK_RIGHT_MODULE_DRIVE_MOTOR,
    BACK_RIGHT_MODULE_STEER_MOTOR, BACK_RIGHT_MODULE_STEER_ENCODER, BACK_RIGHT_MODULE_STEER_OFFSET);
    }
}
