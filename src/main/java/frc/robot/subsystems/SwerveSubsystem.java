// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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
import frc.robot.lib.Swerve.SwerveModuleState2;
import frc.robot.subsystems.modules.PigeonModule;
import frc.robot.subsystems.modules.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  private SwerveDriveOdometry swerveOdometry;
  private SwerveDrivePoseEstimator poseEstimator;
  private SwerveModule[] SwerveMods;
  private PigeonModule m_Pigeon;
  private Field2d field;
  private SwerveModule modules;

  public SwerveSubsystem() {
    // By default we use a Pigeon for our gyroscope. But if you use another
    // gyroscope, like a NavX, you can change this.
    // The important thing about how you configure your gyroscope is that rotating
    // the robot counter-clockwise should
    // cause the angle reading to increase until it wraps back over to zero.

    //Creating the new instance of the pigeon module
    m_Pigeon = PigeonModule.getPigeonModule();

    //Calling the zeroGyro method in order to reset the Gyroscope of the Pigeon for the SwerveDrive
    zeroGyro();

    //Creating the SwerveModules with the four modules of the robot
    SwerveMods = new SwerveModule[] {
        new SwerveModule(0, Ports.frontLeftModule0.constants),
        new SwerveModule(1, Ports.frontRightModule1.constants),
        new SwerveModule(2, Ports.backLeftModule2.constants),
        new SwerveModule(3, Ports.backRightModule3.constants)
    };

    //Creating the odometry of the robot
    swerveOdometry = new SwerveDriveOdometry(Setting.mKinematics, getYaw(), getPositions());
    
    //Creating the position estimation of the SwerveDrive
    poseEstimator = new SwerveDrivePoseEstimator(Setting.mKinematics, getYaw(),
        getPositions(), new Pose2d());

    //Creating and putting the field for Charged Up onto to the SmartDashboard
    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  /*Creating a method for the drive command of the SwerveDrive
  *Translation refers to the robot going on its y axis
  *Rotation referes to the robot z axis
  *Feild relative refers to the robot heading to the orientation of the feild
  *Open loop refers to the control system is independent and has non feedback towards the SwerveDrive
  */
  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Setting.mKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
        Setting.maxVelocityMetersPerSecond);

    for (SwerveModule mod : SwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  //States of the four modules of the SwerveDrive and the desiredState of the optimal location the module need to be in
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Setting.maxVelocityMetersPerSecond);

    for (SwerveModule mod : SwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false); // false
    }
  }

  //Creates a method of the posiion of the SwerveDrive for Odometry
  public Pose2d getPose() {
    SmartDashboard.putNumber("pose X", swerveOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("pose Y", swerveOdometry.getPoseMeters().getY());
    SmartDashboard.putNumber("gyro angle", m_Pigeon.getYaw());
    return swerveOdometry.getPoseMeters();
  }

  //Creates a method to reset the Odometry on the 2d feild
  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  //Creates the SwerveDrive module states of the four modules of the robot
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : SwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  //Creates the method of the SwerveDrive four modules positons
  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : SwerveMods) {
      // SmartDashboard.putNumber("position: module " + mod.moduleNumber,
      // mod.getPosition().distanceMeters);//FIXME put back later if needed
      /// SmartDashboard.putNumber("angle: module " + mod.moduleNumber,
      // mod.getPosition().angle.getDegrees());
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  //Ressetting the Pigeon Gyroscope in order for feild centric driving
  public void zeroGyro() {
    m_Pigeon.setYaw(0.0);
    System.out.println("Feild Centric Activate :)))");
  }

  //Method to get the rotation of the SwerveDrive heading
  public Rotation2d getYaw() {
    return (Ports.Gyro.invertGyro)
        ? Rotation2d.fromDegrees(360 - m_Pigeon.getYaw())
        : Rotation2d.fromDegrees(m_Pigeon.getYaw());
  }
  /**
   * Point all modules toward the robot center, thus making the robot very difficult to move. Forcing the robot to keep
   * the current pose.
   */
  public void lockPose()
  {
    for (SwerveModule swerveModule : SwerveMods)
    {
      swerveModule.setDesiredState(
          new SwerveModuleState2(0, modules.getAngle(), 0), true);
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Updates the SwerveDrive odometry of its heading and the positon
    swerveOdometry.update(getYaw(), getPositions());

    //Ressetting the robot position of the SwerveDrive on the feild
    field.setRobotPose(getPose());

    //To get values for the SwerveDrive modules in SmartDashboard
    for (SwerveModule mod : SwerveMods) {
      // TODO if need some values for later
      // SmartDashboard.putNumber(
      // "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      // SmartDashboard.putNumber(
      // "Mod " + mod.moduleNumber + " Integrated",
      // mod.getState().angle.getDegrees());
      // SmartDashboard.putNumber(
      // "Mod " + mod.moduleNumber + " Velocity",
      // mod.getState().speedMetersPerSecond);
    }
  }
}
