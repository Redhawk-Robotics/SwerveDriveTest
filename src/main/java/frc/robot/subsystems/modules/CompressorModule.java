package frc.robot.subsystems.modules;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.constants.Setting;

public class CompressorModule {
    
    private final Compressor compressor;
    private static CompressorModule singleton;

    public CompressorModule() {
        this.compressor = new Compressor(PneumaticsModuleType.REVPH);
    }

    public static CompressorModule getCompressorModule() {
        if (singleton == null) {
            singleton = new CompressorModule();
        }
        return singleton;
    }

    public void enableAnalog() {
        compressor.enableAnalog(Setting.intakePneumatic.minPressure, Setting.intakePneumatic.maxPressure);
    }

    public boolean isEnabled() {
        return compressor.isEnabled();
    }

    public double getPressure() {
        return compressor.getPressure();
    }
}
