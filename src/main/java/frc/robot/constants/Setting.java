package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public interface Setting {
    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;    

    public interface CurrentLimit{

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 40;
    public static final int driveContinuousCurrentLimit = 40;

    
    }
    public interface RobotCharacterizations{
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(28.5);
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(28.5);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        /** SDS MK4i l1 - 8.14 : 1 */
        /** SDS MK4i l2 - 6.75 : 1 */
        /** SDS MK4i l3 - 6.12 : 1 */
        ///public static final double driveGearRatio = (50.0/14.0) * (17.0/27.0) * (45.0/15.0); // 6.75:1
        public static final double driveGearRatio = (6.75 / 1.0);

        /** SDS MK4i l1 - (150 / 7) : 1 */
        /** SDS MK4i l2 - (150 / 7) : 1 */
        /** SDS MK4i l3 - (150 / 7) : 1 */
        public static final double angleGearRatio = (150.0 / 7.0); //Same for all of the MK4i modules 
        
    /* Swerve Kinematics 
    * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
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
    
}
