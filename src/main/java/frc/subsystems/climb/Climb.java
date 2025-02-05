package frc.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private static Climb instance;

    public static Climb getInstance() {
        if (instance == null) instance = new Climb();
        return instance;
    }

    private Climb() {

    }
}
