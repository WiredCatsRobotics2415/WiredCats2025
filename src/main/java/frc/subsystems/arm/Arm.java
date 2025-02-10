package frc.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.RuntimeConstants;
import frc.constants.Subsystems.ArmConstants;
import frc.utils.Util;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private ArmFeedforward ff = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);
    private ProfiledPIDController pid = new ProfiledPIDController(ArmConstants.kP, 0.0d, ArmConstants.kD,
        new TrapezoidProfile.Constraints(ArmConstants.veloMax, ArmConstants.accelMax));

    @Getter private Angle goal = Degrees.of(0.0);

    private boolean isCoasting = false;
    private static Arm instance;
    @Getter private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private Arm() {
        pid.setTolerance(ArmConstants.GoalTolerance);
        io = (ArmIO) Util.getIOImplementation(ArmIOReal.class, ArmIOSim.class, ArmIO.class);
        if (RuntimeConstants.TuningMode) {
            ArmCharacterization.enable(this);
        }
    }

    public static Arm getInstance() {
        if (instance == null) instance = new Arm();
        return instance;
    }

    /** Sets the goal height. If goalInches is out of the physical range, it is not set. */
    public void setGoal(Angle goal) {
        if (goal.gt(ArmConstants.MaxDegreesFront) || goal.lt(ArmConstants.MaxDegreesBack)) return;
        this.goal = goal;
        pid.setGoal(new TrapezoidProfile.State(goal.in(Degrees), 0.0d));
    }

    /**
     * @return A command to increase the arm's current goal by one degree. Does not go above max rotations defined in constants.
     */
    public Command increaseGoal() {
        return new RepeatCommand(new InstantCommand(() -> {
            if (goal.gt(ArmConstants.MaxDegreesFront)) {
                goal = ArmConstants.MaxDegreesFront;
                return;
            }
            goal = goal.plus(Degrees.of(0.5));
            this.setGoal(goal);
        }));
    }

    /**
     * @return A command to decrease the arm's current goal by one degree. Does not go below min rotations defined in constants.
     */
    public Command decreaseGoal() {
        return new RepeatCommand(new InstantCommand(() -> {
            if (goal.lt(ArmConstants.MaxDegreesBack)) {
                goal = ArmConstants.MaxDegreesBack;
                return;
            }
            goal = goal.minus(Degrees.of(0.5));
            this.setGoal(goal);
        }));
    }

    public void coast() {
        io.setCoast(true);
        isCoasting = true;
    }

    public void brake() {
        io.setCoast(false);
        isCoasting = false;
    }

    public boolean atGoal() {
        return pid.atSetpoint();
    }

    public double getMeasurement() {
        return Util.linearMap(inputs.throughborePosition, ArmConstants.ThroughboreMin, ArmConstants.ThroughboreMax,
            ArmConstants.MaxDegreesBack.in(Degrees), ArmConstants.MaxDegreesFront.in(Degrees));
    }

    private void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = ff.calculate(setpoint.position, setpoint.velocity);
        double voltOut = output + feedforward;
        if (!isCoasting) {
            io.setVoltage(voltOut);
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        double measurement = getMeasurement();
        useOutput(pid.calculate(measurement), pid.getSetpoint());
    }
}
