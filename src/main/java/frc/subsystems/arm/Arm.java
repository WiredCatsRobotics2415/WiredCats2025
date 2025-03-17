package frc.subsystems.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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
    @Getter private double goalDegrees = 0.0d;
    private double lastMeasurement = 90.0d;
    @Getter private DoubleDifferentiableValue differentiableMeasurementDegrees = new DoubleDifferentiableValue();
    private boolean isCoasting = false;
    private boolean hasResetPidController = false;

    private TuneableArmFF ff = new TuneableArmFF(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA,
        "ArmFF");
    @Getter private TuneableProfiledPIDController pid = new TuneableProfiledPIDController(ArmConstants.kP, 0.0d,
        ArmConstants.kD,
        new TrapezoidProfile.Constraints(ArmConstants.BaseVelocityMax.get(), ArmConstants.BaseAccelerationMax.get()),
        "ArmPID");

    @Getter private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private static Arm instance;

    private DoubleDifferentiableValue elevatorDDV = Elevator.getInstance().getDifferentiableMeasurementInches();
    private TuneableNumber elevatorVelocityMultiplier = new TuneableNumber(0.0d, "Arm/ArmVelocityMultiplier");

    private Arm() {
        pid.setTolerance(ArmConstants.GoalTolerance);
        io = (ArmIO) Util.getIOImplementation(ArmIOReal.class, ArmIOSim.class, new ArmIO() {});
        if (RuntimeConstants.TuningMode) {
            ArmCharacterization.enable(this);
            TuningModeTab.getInstance().addCommand("Run to cintake side",
                runOnce(() -> SuperStructure.getInstance().setArmGoalSafely(ArmConstants.MaxDegreesBack)));
            TuningModeTab.getInstance().addCommand("Run to stow",
                runOnce(() -> SuperStructure.getInstance().setArmGoalSafely(90)));
            TuningModeTab.getInstance().addCommand("Run to scoring side",
                runOnce(() -> SuperStructure.getInstance().setArmGoalSafely(ArmConstants.MinDegreesFront)));
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
    public void setGoal(double goal) {
        if (goal > ArmConstants.MaxDegreesBack || goal < ArmConstants.MinDegreesFront) return;
        this.goalDegrees = goal;
        pid.setGoal(new TrapezoidProfile.State(goal, 0.0d));
    }

    /**
     * @return A command to increase the arm's current goal by one degree. Does not go above max rotations defined in constants.
     */
    public Command increaseGoal() {
        return runOnce(() -> {
            if (goalDegrees > ArmConstants.MaxDegreesBack) {
                goalDegrees = ArmConstants.MaxDegreesBack;
                return;
            }
            this.goalDegrees += 0.5;
        });
    }

    /**
     * @return A command to decrease the arm's current goal by one degree. Does not go below min rotations defined in constants.
     */
    public Command decreaseGoal() {
        return runOnce(() -> {
            if (goalDegrees < ArmConstants.MinDegreesFront) {
                goalDegrees = ArmConstants.MinDegreesFront;
                return;
            }
            this.goalDegrees += 0.5;
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

    public double getMeasurement() { return lastMeasurement; }

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

        lastMeasurement = Algebra.linearMap(inputs.throughborePosition, ArmConstants.ThroughboreMin,
            ArmConstants.ThroughboreMax, ArmConstants.MinDegreesFront, ArmConstants.MaxDegreesBack);
        differentiableMeasurementDegrees.update(lastMeasurement);

        if (!hasResetPidController) {
            pid.reset(new TrapezoidProfile.State(lastMeasurement, 0));
            hasResetPidController = true;
        }
        useOutput(pid.calculate(lastMeasurement), pid.getSetpoint(), lastMeasurement);

        Logger.recordOutput("Arm/Actual", lastMeasurement);
        Logger.recordOutput("Arm/Goal", goalDegrees);
        Logger.recordOutput("Arm/Error", pid.getPositionError());
        Logger.recordOutput("Arm/ActualVelocity", differentiableMeasurementDegrees.getFirstDerivative());
        Logger.recordOutput("Arm/ActualAcceleration", differentiableMeasurementDegrees.getSecondDerivative());
    }
}
