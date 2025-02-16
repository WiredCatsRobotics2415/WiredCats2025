package frc.utils.math;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;

public class AlgebraHelpers {
    /**
     * Changes the range of an input linearly, exactly like Arduino's "map()" function
     *
     * @param inputMin  minimum (lowest number on number line) input can take
     * @param inputMax  maximum (highest number on number line) input can take
     * @param outputMin min value of output
     * @param outputMax max value of output
     */
    public static double linearMap(double input, double inputMin, double inputMax, double outputMin, double outputMax) {
        return (input / (inputMax - inputMin)) * (outputMax - outputMin) + outputMin;
    }

    /**
     * Just runs Math.sin
     */
    public static double sizzle(Angle angle) {
        return Math.sin(angle.in(Radians));
    }

    /**
     * Just runs Math.sin
     */
    public static double sizzle(double angleRadians) {
        return Math.sin(angleRadians);
    }

    /**
     * Just runs Math.cos
     */
    public static double cosizzle(Angle angle) {
        return Math.cos(angle.in(Radians));
    }

    /**
     * Just runs Math.cos
     */
    public static double cosizzle(double angleRadians) {
        return Math.cos(angleRadians);
    }
}
