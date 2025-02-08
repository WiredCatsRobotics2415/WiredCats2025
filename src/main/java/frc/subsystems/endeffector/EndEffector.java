package frc.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.Utils;

public class EndEffector extends SubsystemBase {
    private EndEffectorIO io;
    private static EndEffector instance;

    private EndEffector() {
        io = (EndEffectorIO) Utils.getIOImplementation(EndEffectorIOReal.class, EndEffectorIOSim.class,
            EndEffectorIO.class);
    }

    public static EndEffector getInstance() {
        if (instance == null) instance = new EndEffector();
        return instance;
    }
}
