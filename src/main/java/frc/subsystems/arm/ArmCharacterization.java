package frc.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.subsystems.coralintake.CoralIntake;
import frc.subsystems.elevator.Elevator;
import frc.subsystems.superstructure.SuperStructure;
import frc.utils.tuning.Characterizer;
import frc.utils.tuning.TuningModeTab;

public class ArmCharacterization extends Characterizer {
    private static ArmCharacterization instance;
    private Arm arm;
    private final Velocity<VoltageUnit> quasiSpeed = Volts.of(0.5).div(Second.of(1));
    private final Angle TestSafetyThreshold = Degrees.of(7);

    private ArmCharacterization(Arm arm) {
        this.arm = arm;

        SysIdRoutine sysIDRoutineArm = new SysIdRoutine(
            new SysIdRoutine.Config(quasiSpeed, Volts.of(4), null,
                state -> SignalLogger.writeString("SysIDArmState", state.toString())),
            new SysIdRoutine.Mechanism(output -> arm.getIo().setVoltage(output.in(Volts)), null, arm));

        commands.add(sysIDRoutineArm.dynamic(Direction.kForward).onlyWhile(this::withinSafeThreshold)
            .withName("Arm: Dynamic Forward"));
        commands.add(sysIDRoutineArm.dynamic(Direction.kReverse).onlyWhile(this::withinSafeThreshold)
            .withName("Arm: Dynamic Backward"));
        commands.add(sysIDRoutineArm.quasistatic(Direction.kForward).onlyWhile(this::withinSafeThreshold)
            .withName("Arm: Quasi Forward"));
        commands.add(sysIDRoutineArm.quasistatic(Direction.kReverse).onlyWhile(this::withinSafeThreshold)
            .withName("Arm: Quasi Backward"));

        TuningModeTab.getInstance().addCharacterizer("Arm", this);
    }

    private boolean withinSafeThreshold() {
        Angle measurement = arm.getMeasurement();
        Angle cintakeMeasurement = CoralIntake.getInstance().getPivotAngle();
        Distance elevatorHeight = Elevator.getInstance().getMeasurement();
        return !SuperStructure.getInstance().stateIsValid(elevatorHeight, measurement.plus(TestSafetyThreshold),
            cintakeMeasurement.plus(TestSafetyThreshold))
            && !SuperStructure.getInstance().stateIsValid(elevatorHeight, measurement.minus(TestSafetyThreshold),
                cintakeMeasurement.plus(TestSafetyThreshold));
    }

    public static void enable(Arm armSubsystem) {
        if (instance == null) instance = new ArmCharacterization(armSubsystem);
    }
}
