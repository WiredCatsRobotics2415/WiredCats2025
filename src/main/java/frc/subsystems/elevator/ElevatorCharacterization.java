package frc.subsystems.elevator;

import frc.utils.tuning.Characterizer;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.subsystems.arm.Arm;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;

import com.ctre.phoenix6.SignalLogger;

public class ElevatorCharacterization extends Characterizer {
    private Elevator elevator;
    private Velocity<VoltageUnit> speed;

    private ElevatorCharacterization(Elevator elevator) {
        super("Arm");
        this.elevator = elevator;
        speed = Volts.of(0.5).div(Second.of(1));

        SysIdRoutine sysIDRoutineElevator = new SysIdRoutine(new SysIdRoutine.Config(speed, // (1 V/s)
            Volts.of(4), // Prevent brownout
            null, // (10 s)
            state -> SignalLogger.writeString("SysIDElevatorState", state.toString())),
            new SysIdRoutine.Mechanism(
                output -> elevator.getIo().setVoltage(output.in(Volts)), null,
                elevator));

        commands.add(sysIDRoutineElevator.dynamic(Direction.kForward).withName("Elevator: Dynamic Forward"));
        commands.add(sysIDRoutineElevator.dynamic(Direction.kReverse).withName("Elevator: Dynamic Backward"));
        commands.add(sysIDRoutineElevator.quasistatic(Direction.kForward).withName("Elevator: Quasi Forward"));
        commands.add(sysIDRoutineElevator.quasistatic(Direction.kReverse).withName("Elevator: Quasi Backward"));
    }
}

