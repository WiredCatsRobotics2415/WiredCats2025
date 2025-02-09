package frc.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Subsystems.ArmConstants;
import frc.constants.Subsystems.ElevatorConstants;
import frc.utils.Util;
import frc.utils.Visualizer;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private ArmFeedforward ff = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);
    private ProfiledPIDController pid = new ProfiledPIDController(ArmConstants.kP, 0.0d, ArmConstants.kD,
        new TrapezoidProfile.Constraints(ArmConstants.veloMax, ArmConstants.accelMax));

    @Getter private Angle goalDegrees = Degrees.of(0.0);

    private boolean isCoasting = false;
    private static Arm instance;
    private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private Arm() {
        pid.setTolerance(ArmConstants.GoalTolerance);
        io = (ArmIO) Util.getIOImplementation(ArmIOReal.class, ArmIOSim.class, ArmIO.class);
    }

    public static Arm getInstance() {
        if (instance == null) instance = new Arm();
        return instance;
    }

    /** Sets the goal height. If goalInches is out of the physical range, it is not set. */
    public void setGoal(Angle goalDegrees) {
        if (goalDegrees.gt(ArmConstants.MaxDegreesFront) || goalDegrees.lt(ArmConstants.MaxDegreesBack)) return;
        this.goalDegrees = goalDegrees;
        pid.setGoal(new TrapezoidProfile.State(goalDegrees.in(Degrees), 0.0d));
    }

    /**
     * @return A command to increase the arm's current goal by one degree. Does not go above max rotations defined in constants.
     */
    public Command increaseGoal() {
        return new RepeatCommand(new InstantCommand(() -> {
            if (goalDegrees.gt(ArmConstants.MaxDegreesFront)) {
                goalDegrees = ArmConstants.MaxDegreesFront;
                return;
            }
            goalDegrees = goalDegrees.plus(Degrees.of(0.5));
            this.setGoal(goalDegrees);
        }));
    }

    /**
     * @return A command to decrease the arm's current goal by one degree. Does not go below min rotations defined in constants.
     */
    public Command decreaseGoal() {
        return new RepeatCommand(new InstantCommand(() -> {
            if (goalDegrees.lt(ArmConstants.MaxDegreesBack)) {
                goalDegrees = ArmConstants.MaxDegreesBack;
                return;
            }
            goalDegrees = goalDegrees.minus(Degrees.of(0.5));
            this.setGoal(goalDegrees);
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
        return Util.linearMap(inputs.throughborePosition, ArmConstants.ThroughboreMin,
            ArmConstants.ThroughboreMax, ArmConstants.MaxDegreesBack.in(Degrees),
            ArmConstants.MaxDegreesFront.in(Degrees));
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
