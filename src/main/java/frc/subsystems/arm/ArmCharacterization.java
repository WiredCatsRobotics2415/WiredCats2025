package frc.subsystems.arm;

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

public class ArmCharacterization extends Characterizer {
    private Arm arm;
    private Velocity<VoltageUnit> speed;

    private ArmCharacterization(Arm arm) {
        super("Arm");
        this.arm = arm;
        speed = Volts.of(0.5).div(Second.of(1));

        SysIdRoutine sysIDRoutineArm = new SysIdRoutine(new SysIdRoutine.Config(speed, // (1 V/s)
            Volts.of(4), // Prevent brownout
            null, // (10 s)
            state -> SignalLogger.writeString("SysIDArmState", state.toString())),
            new SysIdRoutine.Mechanism(
                output -> arm.getIo().setVoltage(output.in(Volts)), null,
                arm));

        commands.add(sysIDRoutineArm.dynamic(Direction.kForward).withName("Arm: Dynamic Forward"));
        commands.add(sysIDRoutineArm.dynamic(Direction.kReverse).withName("Arm: Dynamic Backward"));
        commands.add(sysIDRoutineArm.quasistatic(Direction.kForward).withName("Arm: Quasi Forward"));
        commands.add(sysIDRoutineArm.quasistatic(Direction.kReverse).withName("Arm: Quasi Backward"));
    }
}