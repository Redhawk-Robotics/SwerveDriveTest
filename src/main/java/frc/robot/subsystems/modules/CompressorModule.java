package frc.robot.subsystems.modules;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.constants.Setting;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsBase;

public class CompressorModule {

    public Compressor compressor;
    public PneumaticHub pneumaticHub;
    private static CompressorModule singleton;

    public CompressorModule() {
        singleton.pneumaticHub = new PneumaticHub();
        // Why make this then???
        singleton.compressor = ((PneumaticsBase) compressor).makeCompressor();
    }

    public static CompressorModule getCompressorModule() {
        if (singleton == null) {
            singleton = new CompressorModule();
        }
        return singleton;
    }

    public void enableCompressorAnalog(double min, double max) {
        singleton.pneumaticHub.enableCompressorAnalog(min, max);
    }

    public void enableCompressorDigital() {
        singleton.pneumaticHub.enableCompressorDigital();
    }
    public boolean isEnabled() {
        return singleton.pneumaticHub.getCompressor();
    }

    public double getPressure() {
        return singleton.compressor.getPressure();
    }

    // public void setPressureAnalog(double pressure) {
    //     if (pressure == 220) {
    //         compressor.disable();
    //     } else if (pressure >= 60) {
    //         enableAnalog(Setting.compressor.absoluteMinPressure, Setting.compressor.absoluteMaxPressure);
    //     } else {
    //         enableAnalog(Setting.compressor.relativeMinPressure, Setting.compressor.absoluteMaxPressure);
    //     }
    // }
}
