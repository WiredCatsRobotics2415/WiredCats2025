package frc.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.RuntimeConstants;
import frc.constants.Subsystems.ArmConstants;
import frc.subsystems.elevator.Elevator;
import frc.subsystems.superstructure.SuperStructure;
import frc.utils.Util;
import frc.utils.math.Algebra;
import frc.utils.math.DoubleDifferentiableValue;
import frc.utils.math.Trig;
import frc.utils.tuning.TuneableArmFF;
import frc.utils.tuning.TuneableNumber;
import frc.utils.tuning.TuneableProfiledPIDController;
import frc.utils.tuning.TuningModeTab;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    @Getter private Angle goal = Degrees.mutable(0.0d);
    @Getter private DoubleDifferentiableValue differentiableMeasurementDegrees = new DoubleDifferentiableValue();
    private boolean isCoasting = false;
    private boolean hasResetPidController = false;

    private TuneableArmFF ff = new TuneableArmFF(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA,
        "ArmFF");
    @Getter private TuneableProfiledPIDController pid = new TuneableProfiledPIDController(ArmConstants.kP, 0.0d,
        ArmConstants.kD,
        new TrapezoidProfile.Constraints(ArmConstants.BaseVelocityMax, ArmConstants.BaseAccelerationMax), "ArmPID");

    @Getter private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private static Arm instance;

    private DoubleDifferentiableValue elevatorDDV = Elevator.getInstance().getDifferentiableMeasurementInches();
    private TuneableNumber elevatorVelocityMultiplier = new TuneableNumber(0.0d, "Arm/ArmVelocityMultiplier");

    private Arm() {
        pid.setTolerance(ArmConstants.GoalTolerance.in(Degrees));
        io = (ArmIO) Util.getIOImplementation(ArmIOReal.class, ArmIOSim.class, new ArmIO() {});
        if (RuntimeConstants.TuningMode) {
            ArmCharacterization.enable(this);
            TuningModeTab.getInstance().addCommand("Run to cintake side",
                runOnce(() -> SuperStructure.getInstance().setArmGoalSafely(ArmConstants.MaxDegreesBack.angle())));
            TuningModeTab.getInstance().addCommand("Run to stow",
                runOnce(() -> SuperStructure.getInstance().setArmGoalSafely(Degrees.of(90))));
            TuningModeTab.getInstance().addCommand("Run to scoring side",
                runOnce(() -> SuperStructure.getInstance().setArmGoalSafely(ArmConstants.MinDegreesFront.angle())));
            TuningModeTab.getInstance().addCommand("Toggle Arm Coast Mode", runOnce(() -> {
                if (isCoasting)
                    brake();
                else
                    coast();
            }));
        }
    }

    public static Arm getInstance() {
        if (instance == null) instance = new Arm();
        return instance;
    }

    /** Sets the goal height. If goal is out of the physical range, it is not set. */
    public void setGoal(Angle goal) {
        if (goal.gt(ArmConstants.MaxDegreesBack.angle()) || goal.lt(ArmConstants.MinDegreesFront.angle())) return;
        this.goal = goal;
        pid.setGoal(new TrapezoidProfile.State(goal.in(Degrees), 0.0d));
    }

    /**
     * @return A command to increase the arm's current goal by one degree. Does not go above max rotations defined in constants.
     */
    public Command increaseGoal() {
        return runOnce(() -> {
            if (goal.gt(ArmConstants.MaxDegreesBack.angle())) {
                goal = ArmConstants.MaxDegreesBack.angle();
                return;
            }
            goal = goal.plus(Degrees.of(0.5));
            this.setGoal(goal);
        });
    }

    /**
     * @return A command to decrease the arm's current goal by one degree. Does not go below min rotations defined in constants.
     */
    public Command decreaseGoal() {
        return runOnce(() -> {
            if (goal.lt(ArmConstants.MinDegreesFront.angle())) {
                goal = ArmConstants.MinDegreesFront.angle();
                return;
            }
            goal = goal.minus(Degrees.of(0.5));
            this.setGoal(goal);
        });
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
        return pid.atGoal();
    }

    public Angle getMeasurement() {
        return Degrees.of(Algebra.linearMap(inputs.throughborePosition, ArmConstants.ThroughboreMin.get(),
            ArmConstants.ThroughboreMax.get(), ArmConstants.MinDegreesFront.in(Degrees),
            ArmConstants.MaxDegreesBack.in(Degrees)));
    }

    private void useOutput(double output, TrapezoidProfile.State setpoint, double measurementDegrees) {
        double feedforward = ff.calculate(Units.degreesToRadians(setpoint.position), setpoint.velocity);
        double voltOut = output + feedforward;

        double elevatorVelocity = elevatorDDV.getFirstDerivative();
        if (elevatorVelocity > 0) {
            double toAdd = (elevatorVelocityMultiplier.get() * elevatorVelocity *
                Trig.cosizzle(Units.degreesToRadians(setpoint.position)));
            if (measurementDegrees < 90 && pid.getPositionError() > 0) {
                voltOut += toAdd;
            }
            if (measurementDegrees > 90 && pid.getPositionError() < 0) {
                voltOut -= toAdd;
            }
        }

        if (!isCoasting) {
            io.setVoltage(voltOut);
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        double measurementDegrees = getMeasurement().in(Degrees);
        differentiableMeasurementDegrees.update(measurementDegrees);

        if (!hasResetPidController) {
            pid.reset(new TrapezoidProfile.State(measurementDegrees, 0));
            hasResetPidController = true;
        }
        useOutput(pid.calculate(measurementDegrees), pid.getSetpoint(), measurementDegrees);

        Logger.recordOutput("Arm/Goal", goal);
        Logger.recordOutput("Arm/Error", pid.getPositionError());
        Logger.recordOutput("Arm/ActualVelocity", differentiableMeasurementDegrees.getFirstDerivative());
        Logger.recordOutput("Arm/ActualAcceleration", differentiableMeasurementDegrees.getSecondDerivative());
    }
}
