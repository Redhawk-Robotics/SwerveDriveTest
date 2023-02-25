package frc.robot.subsystems.modules;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.constants.Ports;

public class PiegeonModule {

    private final Pigeon2 m_Pigeon;
    private static PiegeonModule singleton;

    private PiegeonModule() {
        this.m_Pigeon = new Pigeon2(Ports.Gyro.DRIVETRAIN_PIGEON_ID);
    }
    
    public static PiegeonModule getPigeonModule() {
        if (singleton == null) {
            singleton = new PiegeonModule();
        }
        return singleton;
    }

    public void setYaw(double yaw) {
        this.m_Pigeon.setYaw(yaw);
    }

    public double getYaw() {
        return this.m_Pigeon.getYaw();
    }
}
