package frc.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.constants.Subsystems.ElevatorConstants;
import frc.utils.tuning.Characterizer;
import frc.utils.tuning.TuningModeTab;

public class ElevatorCharacterization extends Characterizer {
    private static ElevatorCharacterization instance;
    private Elevator elevator;
    private final Velocity<VoltageUnit> quasiSpeed = Volts.of(0.5).div(Second.of(1));
    private final Distance TestSafetyThreshold = Inches.of(3);

    private ElevatorCharacterization(Elevator elevator) {
        this.elevator = elevator;

        SysIdRoutine sysIDRoutineElevator = new SysIdRoutine(new SysIdRoutine.Config(quasiSpeed, // (1 V/s)
            Volts.of(4), // Prevent brownout
            null, // (10 s)
            state -> SignalLogger.writeString("SysIDElevatorState", state.toString())),
            new SysIdRoutine.Mechanism(output -> elevator.getIo().setVoltage(output.in(Volts)), null, elevator));

        commands.add(sysIDRoutineElevator.dynamic(Direction.kForward).onlyWhile(this::withinSafeThreshold)
            .withName("Elevator: Dynamic Forward"));
        commands.add(sysIDRoutineElevator.dynamic(Direction.kReverse).onlyWhile(this::withinSafeThreshold)
            .withName("Elevator: Dynamic Backward"));
        commands.add(sysIDRoutineElevator.quasistatic(Direction.kForward).onlyWhile(this::withinSafeThreshold)
            .withName("Elevator: Quasi Forward"));
        commands.add(sysIDRoutineElevator.quasistatic(Direction.kReverse).onlyWhile(this::withinSafeThreshold)
            .withName("Elevator: Quasi Backward"));

        TuningModeTab.getInstance().addCharacterizer("Elevator", this);
    }

    private boolean withinSafeThreshold() {
        Distance measurement = elevator.getMeasurement();
        return measurement.minus(TestSafetyThreshold).gte(ElevatorConstants.MinHeightInches)
            || measurement.plus(TestSafetyThreshold).lte(ElevatorConstants.MaxHeightInches);
    }

    public static void enable(Elevator elevatorSubsystem) {
        if (instance == null) instance = new ElevatorCharacterization(elevatorSubsystem);
    }
}
