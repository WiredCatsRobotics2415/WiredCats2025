package frc.subsystems.arm;

public class Arm {
    private static Arm instance;

    private Arm() {

    }

    public static Arm getInstance() {
        if (instance == null) instance = new Arm();
        return instance;
    }

    public double getAngleDegrees() { return 0; }
}
