package frc.robot.subsystems.modules;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.constants.Setting;

public class CompressorModule {
    
    private final Compressor compressor;
    private static CompressorModule singleton;

    public CompressorModule() {
        this.compressor = new Compressor(1,PneumaticsModuleType.REVPH);
    }

    public static CompressorModule getCompressorModule() {
        if (singleton == null) {
            singleton = new CompressorModule();
        }
        return singleton;
    }

    public void enableAnalog(double min, double max) {
        compressor.enableAnalog(min, max);
    }

    public boolean isEnabled() {
        return compressor.isEnabled();
    }

    public double getPressure() {
        return compressor.getPressure();
    }
    public void enableDigital(){
        compressor.enableDigital();
    }

    public void disableCompressor(){
        compressor.disable();
    }

    // public void setPressureAnalog(double pressure) {
    //     if (pressure == 220) {
    //         compressor.disable();
    //     } else if (pressure >= 60) {
    //         enableAnalog(Setting.compressor.absoluteMinPressure, Setting.compressor.absoluteMaxPressure);
    //     } else {
    //         enableAnalog(Setting.compressor.relativeMinPressure,Setting.compressor.absoluteMaxPressure);
    //     }
    // }
}
