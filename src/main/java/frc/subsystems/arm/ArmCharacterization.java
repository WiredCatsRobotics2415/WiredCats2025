package frc.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.constants.Subsystems.ArmConstants;
import frc.utils.tuning.Characterizer;
import frc.utils.tuning.TuningModeTab;

public class ArmCharacterization extends Characterizer {
    private static ArmCharacterization instance;
    private Arm arm;
    private final Velocity<VoltageUnit> quasiSpeed = Volts.of(0.25).div(Second.of(1));
    private final Angle TestSafetyThreshold = Degrees.of(7);

    private ArmCharacterization(Arm arm) {
        this.arm = arm;

        SysIdRoutine sysIDRoutineArm = new SysIdRoutine(
            new SysIdRoutine.Config(quasiSpeed, Volts.of(2.5), null,
                state -> SignalLogger.writeString("SysIDArmState", state.toString())),
            new SysIdRoutine.Mechanism(output -> arm.getIo().setVoltage(output.in(Volts)), null, arm));

        commands.add(sysIDRoutineArm.dynamic(Direction.kForward).onlyWhile(this::willNotHitFront)
            .withName("Arm: Dynamic Forward"));
        commands.add(sysIDRoutineArm.dynamic(Direction.kReverse).onlyWhile(this::willNotHitBack)
            .withName("Arm: Dynamic Backward"));
        commands.add(sysIDRoutineArm.quasistatic(Direction.kForward).onlyWhile(this::willNotHitFront)
            .withName("Arm: Quasi Forward"));
        commands.add(sysIDRoutineArm.quasistatic(Direction.kReverse).onlyWhile(this::willNotHitBack)
            .withName("Arm: Quasi Backward"));

        TuningModeTab.getInstance().addCharacterizer("Arm", this);
    }

    private boolean willNotHitFront() {
        Angle measurement = arm.getMeasurement();
        return measurement.minus(TestSafetyThreshold).lte(ArmConstants.MinDegreesFront.angle());
    }

    private boolean willNotHitBack() {
        Angle measurement = arm.getMeasurement();
        return measurement.plus(TestSafetyThreshold).gte(ArmConstants.MaxDegreesBack.angle());
    }

    public static void enable(Arm armSubsystem) {
        if (instance == null) instance = new ArmCharacterization(armSubsystem);
    }
}
