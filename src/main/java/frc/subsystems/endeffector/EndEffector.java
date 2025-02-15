package frc.subsystems.endeffector;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Subsystems.EndEffectorConstants;
import frc.utils.Utils;

public class EndEffector extends SubsystemBase {
    private EndEffectorIO io;
    private static EndEffector instance;

    private boolean hasCoral = false;

    // stores the direction which will intake algae and outtake coral (1 is forwards, -1 is reverse)
    private int AlgaeIntakeDirection = -1; 

    private AnalogInput coralSensor;

    private EndEffector() {
        io = (EndEffectorIO) Utils.getIOImplementation(EndEffectorIOReal.class, EndEffectorIOSim.class,
            EndEffectorIO.class);

        coralSensor = new AnalogInput(EndEffectorConstants.EndEffectorIR);
    }

    public static EndEffector getInstance() {
        if (instance == null) instance = new EndEffector();
        return instance;
    }

    // DON'T HAVE TO INVERT MOTOR--JUST SET TO NEGATIVE

    // use infrared sensor to check if coral has been intook??? (idk grammar)
}
