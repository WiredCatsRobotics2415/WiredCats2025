package frc.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.utils.Utils;
import frc.constants.Subsystems.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
    private EndEffectorIO io;
    private static EndEffector instance;

    private boolean hasCoral = false; 
    // stores if running the motor forwards will intake algae and outtake coral
    private boolean ForwardAlgaeIntakeCoralOuttake = true; 

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
