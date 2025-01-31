package frc.utils;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Measurements.MotorConstants;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map.Entry;
import java.util.function.Supplier;

/**
 * What exactly does this class do? It constantly monitors motors for torque "spikes", where torque increases suddenly, and in such an event, a method to stop the subsystem completely is called and an alert is shown
 */
public class TorqueSafety {
    private static TorqueSafety instance;

    private LinkedHashMap<Supplier<Current>, Command> currentSuppliersAndSubsystems = new LinkedHashMap<Supplier<Current>, Command>();
    private ArrayList<Double> lastCurrents = new ArrayList<Double>();
    private double lastTime;

    private TorqueSafety() {
        lastTime = Timer.getFPGATimestamp();
    }

    public static TorqueSafety getInstance() {
        if (instance == null) instance = new TorqueSafety();
        return instance;
    }

    public void periodic() {
        int i = 0;
        for (Entry<Supplier<Current>, Command> entry : currentSuppliersAndSubsystems.entrySet()) {
            double current = entry.getKey().get().in(Amps);
            double currentDifference = current - lastCurrents.get(i);
            double time = Timer.getFPGATimestamp();
            double timeDifference = time - lastTime;
            if ((currentDifference / timeDifference) > MotorConstants.UniversalTorqueCutoffCurrentSeconds) {
                entry.getValue().schedule();
                String statusString = "TORQUE SAFETY TRIPPED BY " + entry.getValue().getName()
                    + " with current difference " + currentDifference;
                new Alert(statusString, AlertType.kError).set(true);
                System.out.println(statusString);
                System.out.println("Robot code must be restarted.");
            }
            lastTime = time;
            lastCurrents.set(i, current);
            i++;
        }
    }

    public void addMotor(Supplier<Current> currentSupplier, Command stopSubsystem) {
        currentSuppliersAndSubsystems.put(currentSupplier, stopSubsystem);
        lastCurrents.add(currentSupplier.get().in(Amps));
    }
}
