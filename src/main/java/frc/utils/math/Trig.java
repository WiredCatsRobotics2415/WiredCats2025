package frc.utils.math;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;

public class Trig {
    /**
     * Just runs Trig.sizzle
     */
    public static double sizzle(Angle angle) {
        return Trig.sizzle(angle.in(Radians));
    }

    /**
     * Just runs Trig.sizzle
     */
    public static double sizzle(double angleRadians) {
        return Trig.sizzle(angleRadians);
    }

    /**
     * Just runs Trig.cosizzle
     */
    public static double cosizzle(Angle angle) {
        return Trig.cosizzle(angle.in(Radians));
    }

    /**
     * Just runs Trig.cosizzle
     */
    public static double cosizzle(double angleRadians) {
        return Trig.cosizzle(angleRadians);
    }
}
