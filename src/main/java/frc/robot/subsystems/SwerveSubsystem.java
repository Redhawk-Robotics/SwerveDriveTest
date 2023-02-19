// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.Setting;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  private SwerveDriveOdometry swerveOdometry;
  // private SwerveDrivePoseEstimator poseEstimator;
  private SwerveModule[] SwerveMods;
  private Pigeon2 m_Pigeon;
  private Field2d field;
  
  public SwerveSubsystem() {
  // By default we use a Pigeon for our gyroscope. But if you use another
  // gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating
  // the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  m_Pigeon = new Pigeon2(Ports.Gyro.DRIVETRAIN_PIGEON_ID);
  zeroGyro();
  
  SwerveMods = new SwerveModule[] {
    new SwerveModule(0, Ports.FRONT_LEFT_MODULE0.constants),
    new SwerveModule(1, Ports.FRONT_RIGHT_MODULE1.constants),
    new SwerveModule(2, Ports.BACK_LEFT_MODULE2.constants),
    new SwerveModule(3, Ports.BACK_RIGHT_MODULE3.constants)
};

swerveOdometry = new SwerveDriveOdometry(Setting.M_KINEMATICS, getYaw(), getPositions());
// poseEstimator = new
// SwerveDrivePoseEstimator(Ports.SwerveDriveChars.m_kinematics, getYaw(),
// getPositions(), new Pose2d(), STATE_STD_DEVS, VISION_MEASUREMENT_STD_DEVS);

field = new Field2d();
SmartDashboard.putData("Field", field);
  }
public void drive(
  Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
SwerveModuleState[] swerveModuleStates = Setting.M_KINEMATICS.toSwerveModuleStates(
    fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), translation.getY(), rotation, getYaw())
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
    Setting.MAX_VELOCITY_METERS_PER_SECOND);

for (SwerveModule mod : SwerveMods) {
  mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
  }
}
/* Used by SwerveControllerCommand in Auto */
public void setModuleStates(SwerveModuleState[] desiredStates) {
  SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Setting.MAX_VELOCITY_METERS_PER_SECOND);

  for (SwerveModule mod : SwerveMods) {
    mod.setDesiredState(desiredStates[mod.moduleNumber], false); // false
  }
}
public Pose2d getPose() {
  SmartDashboard.putNumber("pose X", swerveOdometry.getPoseMeters().getX());
  SmartDashboard.putNumber("pose Y", swerveOdometry.getPoseMeters().getY());
  SmartDashboard.putNumber("gyro angle", m_Pigeon.getYaw());
  return swerveOdometry.getPoseMeters();
}

public void resetOdometry(Pose2d pose) {
  swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
}

public SwerveModuleState[] getStates() {
  SwerveModuleState[] states = new SwerveModuleState[4];
  for (SwerveModule mod : SwerveMods) {
    states[mod.moduleNumber] = mod.getState();
  }
  return states;
}
public SwerveModulePosition[] getPositions() {
  SwerveModulePosition[] positions = new SwerveModulePosition[4];
  for (SwerveModule mod : SwerveMods) {
    SmartDashboard.putNumber("position: module " + mod.moduleNumber, mod.getPosition().distanceMeters);
    SmartDashboard.putNumber("angle: module " + mod.moduleNumber, mod.getPosition().angle.getDegrees());
    positions[mod.moduleNumber] = mod.getPosition();
  }
  return positions;
}

public void zeroGyro() {
  m_Pigeon.setYaw(0.0);
  System.out.println("Feild Centric Activate :)))");
}

public Rotation2d getYaw() {
  return (Ports.Gyro.invertGyro)
      ? Rotation2d.fromDegrees(360 - m_Pigeon.getYaw())
      : Rotation2d.fromDegrees(m_Pigeon.getYaw());
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveOdometry.update(getYaw(), getPositions());
    field.setRobotPose(getPose());

    for (SwerveModule mod : SwerveMods) {
      //TODO if need some values for later
      // SmartDashboard.putNumber(
      //     "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      // SmartDashboard.putNumber(
      //     "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      // SmartDashboard.putNumber(
      //     "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}
