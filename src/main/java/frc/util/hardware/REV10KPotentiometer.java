package frc.util.hardware;

import edu.wpi.first.wpilibj.AnalogInput;
import lombok.Getter;
import lombok.Setter;

public class REV10KPotentiometer {
    private AnalogInput input;

    @Getter
    @Setter private double minimumVolt;

    @Getter
    @Setter private double maximumVolt;

    private double fullVoltRange;
    private double fullAngleRange;

    public REV10KPotentiometer(int port, double minimumVolt, double maximumVolt,
        double fullAngleRange) {
        this.input = new AnalogInput(port);
        this.minimumVolt = minimumVolt;
        this.maximumVolt = maximumVolt;
        this.fullVoltRange = maximumVolt - minimumVolt;
        this.fullAngleRange = fullAngleRange;
    }

    public double getDegrees() {
        return fullAngleRange -
            (((input.getAverageVoltage() - minimumVolt) / fullVoltRange) * fullAngleRange);
    }

    public double getVoltage() { return input.getAverageVoltage(); }
}
