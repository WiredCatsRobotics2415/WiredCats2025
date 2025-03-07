package frc.subsystems.leds;

import frc.constants.Subsystems.LEDStripConstants.UseableColor;

public interface LEDStripIO {
    public default void setColor(UseableColor color) {};
}
