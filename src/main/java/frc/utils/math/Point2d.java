package frc.utils.math;

import lombok.Getter;

public class Point2d {
    @Getter private double x;
    @Getter private double y;

    public Point2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ")";
    }
}
