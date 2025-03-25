package frc.subsystems.coralintake;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.RuntimeConstants;
import frc.constants.Subsystems.CoralIntakeConstants;
import frc.utils.Util;
import frc.utils.math.Algebra;
import frc.utils.math.DoubleDifferentiableValue;
import frc.utils.tuning.TuneableArmFF;
import frc.utils.tuning.TuneableProfiledPIDController;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends SubsystemBase {
    private CoralIntakeIO io;
    private CoralIntakeIOInputsAutoLogged inputs = new CoralIntakeIOInputsAutoLogged();
    private static CoralIntake instance;

    private TuneableArmFF ff = new TuneableArmFF(CoralIntakeConstants.kS, CoralIntakeConstants.kG,
        CoralIntakeConstants.kV, CoralIntakeConstants.kA, "CoralIntakeFF");
    @Getter private TuneableProfiledPIDController pid = new TuneableProfiledPIDController(CoralIntakeConstants.kP, 0,
        CoralIntakeConstants.kD, new Constraints(CoralIntakeConstants.SlapdownVelocityMax.get(),
            CoralIntakeConstants.SlapdownAccelerationMax.get()),
        "CoralIntakePID");

    @Getter private double goal = 0.0;
    @Getter private DoubleDifferentiableValue differentiableMeasurementDegrees = new DoubleDifferentiableValue();
    private double lastMeasurement = 0.0;
    @Getter private boolean intaking = false;
    @Getter private boolean outtaking = false;
    private boolean hasResetPidController = false;

    private CoralIntake() {
        io = (CoralIntakeIO) Util.getIOImplementation(CoralIntakeIOReal.class, CoralIntakeIOSim.class,
            new CoralIntakeIO() {});
        pid.setTolerance(CoralIntakeConstants.BaseGoalTolerance);

        if (RuntimeConstants.TuningMode) {
            CoralIntakeCharacterizer.createInstance(this);
        }
    }

    public static CoralIntake getInstance() {
        if (instance == null) instance = new CoralIntake();
        return instance;
    }

    public Command slapdown() {
        return runOnce(() -> {
            this.goal = 0;
            pid.setGoal(0);
            io.setPivotVoltage(-CoralIntakeConstants.BlindMoveVoltage);
        }).andThen(Commands.waitSeconds(CoralIntakeConstants.BlindMoveTime)).andThen(runOnce(() -> {
            io.setPivotVoltage(0);
        }));
    }

    public Command stow() {
        return runOnce(() -> {
            this.goal = CoralIntakeConstants.StowAngle.get();
            pid.setGoal(CoralIntakeConstants.StowAngle.get());
            io.setPivotVoltage(CoralIntakeConstants.BlindMoveVoltage);
        }).andThen(Commands.waitSeconds(CoralIntakeConstants.BlindMoveTime)).andThen(runOnce(() -> {
            io.setPivotVoltage(0);
        }));
    }

    public void setPivotGoal(double goal) {
        if (goal <= CoralIntakeConstants.MaxAngle.get() && goal >= CoralIntakeConstants.GroundAngle.get()) {
            this.goal = goal;
            pid.setGoal(goal);
        }
    }

    // public void setPivotGoal(double goal) {
    // this.goal = goal;
    // pid.setGoal(goal);
    // if (goal > 45) {
    // stow().schedule();
    // } else {
    // slapdown().schedule();
    // }
    // }

    public double getPivotAngle() { return lastMeasurement; }

    public Command toggleIntake() {
        return runOnce(() -> {
            if (!intaking) {
                io.setIntakePower(CoralIntakeConstants.IntakeSpeed.get());
                intaking = true;
            } else {
                io.setIntakePower(0);
                intaking = false;
            }
            outtaking = false;
        });
    }

    public Command toggleOuttake() {
        return runOnce(() -> {
            if (!outtaking) {
                io.setIntakePower(CoralIntakeConstants.OuttakeSpeed.get());
                outtaking = true;
            } else {
                io.setIntakePower(0);
                outtaking = false;
            }
            intaking = false;
        });
    }

    public Command turnOffRollers() {
        return runOnce(() -> {
            io.setIntakePower(0);
            intaking = false;
            outtaking = false;
        });
    }

    public boolean pivotAtGoal() {
        return pid.atGoal();
        // return MathUtil.isNear(goal, lastMeasurement, pid.getPositionTolerance());
    }

    private void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = ff.calculate(Units.degreesToRadians(setpoint.position), setpoint.velocity);
        double voltOut = output + feedforward;

        io.setPivotVoltage(voltOut);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("CoralIntake", inputs);

        lastMeasurement = Algebra.linearMap(inputs.throughborePosition, CoralIntakeConstants.ThroughboreMin,
            CoralIntakeConstants.ThroughboreMax, CoralIntakeConstants.GroundAngle.get(),
            CoralIntakeConstants.MaxAngle.get());
        differentiableMeasurementDegrees.update(lastMeasurement);

        if (!hasResetPidController) {
            pid.reset(new TrapezoidProfile.State(lastMeasurement, 0));
            hasResetPidController = true;
        }
        useOutput(pid.calculate(lastMeasurement), pid.getSetpoint());

        Logger.recordOutput("CoralIntake/Actual", lastMeasurement);
        Logger.recordOutput("CoralIntake/Goal", goal);
        Logger.recordOutput("CoralIntake/Error", pid.getPositionError());
        Logger.recordOutput("CoralIntake/ActualVelocity", differentiableMeasurementDegrees.getFirstDerivative());
        Logger.recordOutput("CoralIntake/ActualAcceleration", differentiableMeasurementDegrees.getSecondDerivative());
    }

    public CoralIntakeIO getIo() { return io; }
}
