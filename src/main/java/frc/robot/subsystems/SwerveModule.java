package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.constants.Setting;
import frc.robot.lib.SwerveModuleConstants;
import frc.robot.lib.math.OnboardModuleState;
import frc.robot.lib.util.CANCoderUtil;
import frc.robot.lib.util.CANSparkMaxUtil;
import frc.robot.lib.util.CANCoderUtil.CCUsage;
import frc.robot.lib.util.CANSparkMaxUtil.Usage;

public class SwerveModule {
public int moduleNumber;

public Rotation2d angleOffset;
private Rotation2d lastAngle;

private final CANSparkMax driveMotor;
private final CANSparkMax angleMotor;

private final RelativeEncoder driveEncoder;
private final RelativeEncoder integratedAngleEncoder;

private final CANCoder angleEncoder;

private final SparkMaxPIDController driveController;
private final SparkMaxPIDController angleController;

SimpleMotorFeedforward feedForward =
new SimpleMotorFeedforward(Setting.DRIVE_KS,Setting.DRIVE_KV,Setting.DRIVE_KA);//FIXME find values

public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle;
    }
public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
    }
private void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
    integratedAngleEncoder.setPosition(absolutePosition);
    }

    private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }
private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(Setting.ANGLE_CONTINOUS_CURRENT_LIMIT);
    angleMotor.setInverted(Setting.ANGLE_MOTOR_INVERT);
    angleMotor.setIdleMode(Setting.ANGLE_NEUTRAL_MODE);
    integratedAngleEncoder.setPositionConversionFactor(Setting.ANGLE_CONVERSION_FACTOR);
    angleController.setP(Setting.ANGLE_KP);
    angleController.setI(Setting.ANGLE_KI);
    angleController.setD(Setting.ANGLE_KD);
    angleController.setFF(Setting.ANGLE_KF);
    angleMotor.enableVoltageCompensation(Setting.VOLTAGE_COMP);
    angleMotor.burnFlash();
    Timer.delay(1);
    resetToAbsolute();
    }

private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Setting.DRIVE_CONTINOUS_CURRENT_LIMIT);
    driveMotor.setIdleMode(Setting.DRIVE_NEUTRAL_MODE);
    driveEncoder.setVelocityConversionFactor(Setting.DRIVE_CONVERSION_VELOCITY_FACTOR);
    driveEncoder.setPositionConversionFactor(Setting.DRIVE_CONVERSION_POSITION_FACTOR);
    driveController.setP(Setting.ANGLE_KP);
    driveController.setI(Setting.ANGLE_KI);
    driveController.setD(Setting.ANGLE_KD);
    driveController.setFF(Setting.ANGLE_KF);
    driveMotor.enableVoltageCompensation(Setting.VOLTAGE_COMP);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
    }
private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
        double percentOutput = desiredState.speedMetersPerSecond / Setting.MAX_VELOCITY_METERS_PER_SECOND;
        driveMotor.set(percentOutput);
    } else {
        driveController.setReference(
            desiredState.speedMetersPerSecond,
            ControlType.kVelocity,
            0,
            feedForward.calculate(desiredState.speedMetersPerSecond));
    }
    }

private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Setting.MAX_VELOCITY_METERS_PER_SECOND * 0.01))
            ? lastAngle
            : desiredState.angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
    }

private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }
public SwerveModulePosition getPosition(){
    //SmartDashboard.putNumber("angleEncoder position " + moduleNumber, angleEncoder.getPosition());
    ///SmartDashboard.putNumber("angleOffset degrees " + moduleNumber, angleOffset.getDegrees());

    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        Rotation2d.fromDegrees(angleEncoder.getPosition() - angleOffset.getDegrees()));
    }  
}
