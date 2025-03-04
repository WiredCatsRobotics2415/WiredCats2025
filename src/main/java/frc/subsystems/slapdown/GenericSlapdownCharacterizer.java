package frc.subsystems.slapdown;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.utils.tuning.Characterizer;
import frc.utils.tuning.TuningModeTab;
import org.littletonrobotics.junction.Logger;

public class GenericSlapdownCharacterizer extends Characterizer {
    private GenericSlapdown slapdown;
    private final Velocity<VoltageUnit> quasiSpeed = Volts.of(0.5).div(Second.of(1));

    private final Angle TestSafetyThreshold = Degrees.of(7);
    private Angle testMaxAngle;
    private Angle testMinAngle;

    private GenericSlapdownCharacterizer(GenericSlapdown slapdown, String name, Angle min, Angle max) {
        this.slapdown = slapdown;
        testMaxAngle = max;
        testMinAngle = min;

        SysIdRoutine sysIDRoutineArm = new SysIdRoutine(
            new SysIdRoutine.Config(quasiSpeed, Volts.of(4), null,
                state -> Logger.recordOutput("SysID" + name + "State", state.toString())),
            new SysIdRoutine.Mechanism(output -> slapdown.getIo().setPivotVoltage(output.in(Volts)), null, slapdown));

        commands.add(sysIDRoutineArm.dynamic(Direction.kForward).onlyWhile(this::withinSafeThreshold)
            .withName(name + ": Dynamic Forward"));
        commands.add(sysIDRoutineArm.dynamic(Direction.kReverse).onlyWhile(this::withinSafeThreshold)
            .withName(name + ": Dynamic Backward"));
        commands.add(sysIDRoutineArm.quasistatic(Direction.kForward).onlyWhile(this::withinSafeThreshold)
            .withName(name + ": Quasi Forward"));
        commands.add(sysIDRoutineArm.quasistatic(Direction.kReverse).onlyWhile(this::withinSafeThreshold)
            .withName(name + ": Quasi Backward"));

        TuningModeTab.getInstance().addCharacterizer(name, this);
    }

    private boolean withinSafeThreshold() {
        Angle measurement = slapdown.getPivotAngle();
        return measurement.minus(TestSafetyThreshold).gte(testMinAngle)
            || measurement.plus(TestSafetyThreshold).lte(testMaxAngle);
    }

    public static GenericSlapdownCharacterizer createInstance(GenericSlapdown slapdown, String name, Angle min,
        Angle max) {
        return new GenericSlapdownCharacterizer(slapdown, name, min, max);
    }
}
