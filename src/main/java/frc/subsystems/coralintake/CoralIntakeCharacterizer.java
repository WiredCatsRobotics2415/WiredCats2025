package frc.subsystems.coralintake;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.constants.Subsystems.CoralIntakeConstants;
import frc.utils.tuning.Characterizer;
import frc.utils.tuning.TuningModeTab;
import org.littletonrobotics.junction.Logger;

public class CoralIntakeCharacterizer extends Characterizer {
    private CoralIntake slapdown;
    private final Velocity<VoltageUnit> quasiSpeed = Volts.of(2.5).div(Second.of(1));

    private final double TestSafetyThreshold = 7;

    private CoralIntakeCharacterizer(CoralIntake slapdown) {
        this.slapdown = slapdown;

        SysIdRoutine sysIDRoutineArm = new SysIdRoutine(
            new SysIdRoutine.Config(quasiSpeed, Volts.of(4), null,
                state -> Logger.recordOutput("SysIDCoralIntakeState", state.toString())),
            new SysIdRoutine.Mechanism(output -> slapdown.getIo().setPivotVoltage(output.in(Volts)), null, slapdown));

        commands.add(sysIDRoutineArm.dynamic(Direction.kForward).withName("Coral Intake: Dynamic Forward"));
        commands.add(sysIDRoutineArm.dynamic(Direction.kReverse).withName("Coral Intake: Dynamic Backward"));
        commands.add(sysIDRoutineArm.quasistatic(Direction.kForward).withName("Coral Intake: Quasi Forward"));
        commands.add(sysIDRoutineArm.quasistatic(Direction.kReverse).withName("Coral Intake: Quasi Backward"));

        TuningModeTab.getInstance().addCharacterizer("CoralIntake", this);
    }

    private boolean willNotHitFront() {
        if (!Characterizer.enableSafety.get()) return true;
        return (slapdown.getPivotAngle() - TestSafetyThreshold) < CoralIntakeConstants.GroundAngle.get();
    }

    private boolean willNotHitBack() {
        if (!Characterizer.enableSafety.get()) return true;
        return (slapdown.getPivotAngle() + TestSafetyThreshold) > CoralIntakeConstants.StowAngle.get();
    }

    private boolean withinSafeThreshold() {
        double measurement = slapdown.getPivotAngle();
        return (measurement - TestSafetyThreshold) < CoralIntakeConstants.GroundAngle.get()
            || (measurement + TestSafetyThreshold) > CoralIntakeConstants.StowAngle.get();
    }

    public static CoralIntakeCharacterizer createInstance(CoralIntake slapdown) {
        return new CoralIntakeCharacterizer(slapdown);
    }
}
