package frc.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.Util;

public class Climb extends SubsystemBase {
    private static Climb instance;

    private ClimbIO io;

    private Climb() {
        io = (ClimbIO) Util.getIOImplementation(ClimbIOReal.class, ClimbIO.class, new ClimbIO() {});
    }

    public static Climb getInstance() {
        if (instance == null) instance = new Climb();
        return instance;
    }

    public Command runForward() {
        return null;
    }
}
