package frc.utils.math;

public record Point2d(double x, double y) {
    public Point2d {}

    @Override
    public String toString() {
        return "(" + x + ", " + y + ")";
    }
}
