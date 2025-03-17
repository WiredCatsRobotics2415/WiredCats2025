package frc.subsystems.slapdown;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.constants.Subsystems.CoralIntakeConstants;
import frc.utils.tuning.Characterizer;
import frc.utils.tuning.TuneableNumber;
import frc.utils.tuning.TuningModeTab;
import org.littletonrobotics.junction.Logger;

public class GenericSlapdownCharacterizer extends Characterizer {
    private GenericSlapdown slapdown;
    private final Velocity<VoltageUnit> quasiSpeed = Volts.of(2.5).div(Second.of(1));

    private final double TestSafetyThreshold = 7;
    private TuneableNumber testMaxAngle;
    private TuneableNumber testMinAngle;

    private GenericSlapdownCharacterizer(GenericSlapdown slapdown, String name, TuneableNumber min,
        TuneableNumber max) {
        this.slapdown = slapdown;
        testMaxAngle = max;
        testMinAngle = min;

        SysIdRoutine sysIDRoutineArm = new SysIdRoutine(
            new SysIdRoutine.Config(quasiSpeed, Volts.of(7), null,
                state -> Logger.recordOutput("SysID" + name + "State", state.toString())),
            new SysIdRoutine.Mechanism(output -> slapdown.getIo().setPivotVoltage(output.in(Volts)), null, slapdown));

        commands.add(sysIDRoutineArm.dynamic(Direction.kForward).withName(name + ": Dynamic Forward")
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        commands.add(sysIDRoutineArm.dynamic(Direction.kReverse).withName(name + ": Dynamic Backward")
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        commands.add(sysIDRoutineArm.quasistatic(Direction.kForward).withName(name + ": Quasi Forward")
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        commands.add(sysIDRoutineArm.quasistatic(Direction.kReverse).withName(name + ": Quasi Backward")
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

        TuningModeTab.getInstance().addCharacterizer(name, this);
    }

    private boolean willNotHitFront() {
        if (!Characterizer.enableSafety.get()) return true;
        return (slapdown.getPivotAngle() - TestSafetyThreshold) < CoralIntakeConstants.GroundAngle.get();
    }

    private boolean willNotHitBack() {
        if (!Characterizer.enableSafety.get()) return true;
        return (slapdown.getPivotAngle() + TestSafetyThreshold) > CoralIntakeConstants.MaxAngle.get();
    }

    private boolean withinSafeThreshold() {
        double measurement = slapdown.getPivotAngle();
        return (measurement - TestSafetyThreshold) > testMinAngle.get()
            || (measurement + TestSafetyThreshold) < testMaxAngle.get();
    }

    public static GenericSlapdownCharacterizer createInstance(GenericSlapdown slapdown, String name, TuneableNumber min,
        TuneableNumber max) {
        return new GenericSlapdownCharacterizer(slapdown, name, min, max);
    }
}
