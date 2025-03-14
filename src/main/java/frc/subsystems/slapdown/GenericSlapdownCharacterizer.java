package frc.subsystems.slapdown;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.constants.Subsystems.ArmConstants;
import frc.utils.tuning.Characterizer;
import frc.utils.tuning.TuneableNumber;
import frc.utils.tuning.TuningModeTab;
import org.littletonrobotics.junction.Logger;

public class GenericSlapdownCharacterizer extends Characterizer {
    private GenericSlapdown slapdown;
    private final Velocity<VoltageUnit> quasiSpeed = Volts.of(0.5).div(Second.of(1));

    private final Angle TestSafetyThreshold = Degrees.of(7);
    private TuneableNumber testMaxAngle;
    private TuneableNumber testMinAngle;

    private GenericSlapdownCharacterizer(GenericSlapdown slapdown, String name, TuneableNumber min,
        TuneableNumber max) {
        this.slapdown = slapdown;
        testMaxAngle = max;
        testMinAngle = min;

        SysIdRoutine sysIDRoutineArm = new SysIdRoutine(
            new SysIdRoutine.Config(quasiSpeed, Volts.of(2), null,
                state -> Logger.recordOutput("SysID" + name + "State", state.toString())),
            new SysIdRoutine.Mechanism(output -> slapdown.getIo().setPivotVoltage(output.in(Volts)), null, slapdown));

        commands.add(sysIDRoutineArm.dynamic(Direction.kForward).onlyWhile(this::willNotHitFront)
            .withName(name + ": Dynamic Forward"));
        commands.add(sysIDRoutineArm.dynamic(Direction.kReverse).onlyWhile(this::willNotHitBack)
            .withName(name + ": Dynamic Backward"));
        commands.add(sysIDRoutineArm.quasistatic(Direction.kForward).onlyWhile(this::willNotHitFront)
            .withName(name + ": Quasi Forward"));
        commands.add(sysIDRoutineArm.quasistatic(Direction.kReverse).onlyWhile(this::willNotHitBack)
            .withName(name + ": Quasi Backward"));

        TuningModeTab.getInstance().addCharacterizer(name, this);
    }

    private boolean willNotHitFront() {
        if (!Characterizer.enableSafety.get()) return true;
        Angle measurement = slapdown.getPivotAngle();
        return measurement.minus(TestSafetyThreshold).lte(ArmConstants.MinDegreesFront.angle());
    }

    private boolean willNotHitBack() {
        if (!Characterizer.enableSafety.get()) return true;
        Angle measurement = slapdown.getPivotAngle();
        return measurement.plus(TestSafetyThreshold).gte(ArmConstants.MaxDegreesBack.angle());
    }

    private boolean withinSafeThreshold() {
        Angle measurement = slapdown.getPivotAngle();
        return measurement.minus(TestSafetyThreshold).gte(testMinAngle.angle())
            || measurement.plus(TestSafetyThreshold).lte(testMaxAngle.angle());
    }

    public static GenericSlapdownCharacterizer createInstance(GenericSlapdown slapdown, String name, TuneableNumber min,
        TuneableNumber max) {
        return new GenericSlapdownCharacterizer(slapdown, name, min, max);
    }
}
