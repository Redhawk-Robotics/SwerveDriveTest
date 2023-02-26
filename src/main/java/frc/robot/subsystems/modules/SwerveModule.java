package frc.robot.subsystems.modules;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
new SimpleMotorFeedforward(Setting.driveKS,Setting.driveKV,Setting.driveKA);//FIXME find values

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
    angleMotor.setSmartCurrentLimit(Setting.angleContinousCurentLimit);
    angleMotor.setInverted(Setting.angleMotorInvert);
    angleMotor.setIdleMode(Setting.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(Setting.angleConversionFactor);
    angleController.setP(Setting.angleKP);
    angleController.setI(Setting.angleKI);
    angleController.setD(Setting.angleKD);
    angleController.setFF(Setting.angleKFF);
    angleMotor.enableVoltageCompensation(Setting.voltageComp);
    angleMotor.burnFlash();
    Timer.delay(1);
    resetToAbsolute();
    }

private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Setting.driveContinousCurrentLimit);
    driveMotor.setIdleMode(Setting.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Setting.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Setting.driveConversionPositionFactor);
    driveController.setP(Setting.driveKP);
    driveController.setI(Setting.driveKI);
    driveController.setD(Setting.driveKD);
    driveController.setFF(Setting.driveKFF);
    driveMotor.enableVoltageCompensation(Setting.voltageComp);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
    }
private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
        double percentOutput = desiredState.speedMetersPerSecond / Setting.maxVelocityMetersPerSecond;
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
        (Math.abs(desiredState.speedMetersPerSecond) <= (Setting.maxVelocityMetersPerSecond * 0.01))
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
    SmartDashboard.putNumber("Encoder position " + moduleNumber, angleEncoder.getPosition());
    SmartDashboard.putNumber("Offset degrees " + moduleNumber, angleOffset.getDegrees());

    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        Rotation2d.fromDegrees(angleEncoder.getPosition() - angleOffset.getDegrees()));
    }  
}
