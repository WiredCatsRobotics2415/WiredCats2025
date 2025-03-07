package frc.subsystems.leds;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.constants.Subsystems.LEDStripConstants;
import frc.constants.Subsystems.LEDStripConstants.UseableColor;

public class LEDStripIOBlinkin implements LEDStripIO {
    private Spark blinkin;

    public LEDStripIOBlinkin() {
        blinkin = new Spark(LEDStripConstants.BlinkinPort);
    }

    @Override
    public void setColor(UseableColor color) {
        blinkin.set(color.sparkPWMLevel);
    }
}
