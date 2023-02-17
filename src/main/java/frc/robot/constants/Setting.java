package frc.robot.constants;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public interface Setting {
    /* Swerve Voltage Compensation */
    public static final double VOLTAGE_COMP = 8.5;    

    //RobotCharacterizations
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(28.5);
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(28.5);
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    
    /* Neutral Modes */
    public static final IdleMode ANGLE_NEUTRAL_MODE = IdleMode.kBrake;//kCoast
    public static final IdleMode DRIVE_NEUTRAL_MODE = IdleMode.kBrake;

    /* Sensor Initialization strategy */
    //public static final SensorInitializationStrategy ANGLE_SENSOR_INIT_STRATEGY = SensorInitializationStrategy.BootToZero;
    //public static final SensorInitializationStrategy DRIVE_SENSOR_INIT_STRATEGY = SensorInitializationStrategy.BootToZero;

    /* Motor Inverts */
    public static final boolean DRIVE_MOTOR_INVERT = false;
    public static final boolean ANGLE_MOTOR_INVERT = false;

    /* Swerve Profiling Values */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0 / 60.0 /
    (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) * 0.10033 * Math.PI;      

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
    Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
    
    /* Swerve Current Limiting */
    public static final int ANGLE_CONTINOUS_CURRENT_LIMIT = 40;//FIXME
    public static final int DRIVE_CONTINOUS_CURRENT_LIMIT = 40;//FIXME

    /* Angle Motor PID Values */
    public static final double ANGLE_KP = 0.01;//FIXME or put 0.01
    public static final double ANGLE_KI = 0.0;//FIXME
    public static final double ANGLE_KD = 0.0;//FIXME
    public static final double ANGLE_KFF = 0.0;//FIXME or put 0.0

    /* Drive Motor PID Values */
    public static final double DRIVE_KP = 0.1;//FIXME
    public static final double DRIVE_KI = 0.0;//FIXME
    public static final double DRIVE_KD = 0.0;//FIXME
    public static final double DRIVE_KFF = 0.0;//FIXME

    /* Drive Motor Characterization Values */
    public static final double DRIVE_KS = 0.0;//FIXME
    public static final double DRIVE_KV = 0.0;//FIXME Volt-seconds per meter (max voltage divided by max speed) 12/MAX SPEED
    public static final double DRIVE_KA = 0.0;//FIXME Volt-seconds^2 per meter (max voltage divided by max accel) 12/max acceleration


    /** SDS MK4i l1 - 8.14 : 1 */       
    /** SDS MK4i l2 - 6.75 : 1 */
    /** SDS MK4i l3 - 6.12 : 1 */
    ///public static final double driveGearRatio = (50.0/14.0) * (17.0/27.0) * (45.0/15.0); // 6.75:1
    public static final double DRIVE_GEAR_RATIO = (6.75 / 1.0);

    /** SDS MK4i l1 - (150 / 7) : 1 */
    /** SDS MK4i l2 - (150 / 7) : 1 */
    /** SDS MK4i l3 - (150 / 7) : 1 */
    public static final double ANGLE_GEAR_RATIO = (150.0 / 7.0); //Same for all of the MK4i modules 

    /* Drive Motor Conversion Factors */
    public static final double DRIVE_CONVERSION_POSITION_FACTOR =
    (WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO;
    public static final double DRIVE_CONVERSION_VELOCITY_FACTOR = DRIVE_CONVERSION_POSITION_FACTOR / 60.0;
    public static final double ANGLE_CONVERSION_FACTOR = 360.0 / ANGLE_GEAR_RATIO;

    /*
    * These values are used by the drive falcon to ramp in open loop and closed
    * loop driving.
    * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
    */
    // public static final double OPEN_LOOP_RAMP = 0.25;
    // public static final double CLOSED_LOOP_RAMP = 0.0;

    // /* Swerve Current Limiting */
    // private static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
    // private static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
    // private static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
    // private static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;
    // public static final SupplyCurrentLimitConfiguration ANGLE_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(
    //         ANGLE_ENABLE_CURRENT_LIMIT, ANGLE_CONTINUOUS_CURRENT_LIMIT, ANGLE_PEAK_CURRENT_LIMIT,
    //         ANGLE_PEAK_CURRENT_DURATION);

    // private static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
    // private static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
    // private static final double DRIVE_PEAL_CURRENT_DURATION = 0.1;
    // private static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;
    // public static final SupplyCurrentLimitConfiguration DRIVE_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(
    //         DRIVE_ENABLE_CURRENT_LIMIT, DRIVE_CONTINUOUS_CURRENT_LIMIT, DRIVE_PEAK_CURRENT_LIMIT,
    //         DRIVE_PEAL_CURRENT_DURATION);

    /*------- CANcoder Config ------- */
    /* Angle Encoder Invert */
    public static final boolean CAN_CODER_INVERT = false;
    
    // public static final AbsoluteSensorRange CANCODER_ABSOLUTE_SENSOR_RANGE = AbsoluteSensorRange.Unsigned_0_to_360;
    // public static final SensorInitializationStrategy CANCODER_SENSOR_INIT_STRATEGY = SensorInitializationStrategy.BootToAbsolutePosition;
    // public static final SensorTimeBase CANCODER_SENSOR_TIME_BASE = SensorTimeBase.PerSecond;        

    /* Swerve Kinematics 
    * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public final SwerveDriveKinematics M_KINEMATICS = new SwerveDriveKinematics(
        // Front left
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Front right
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back left
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back right
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );
}
