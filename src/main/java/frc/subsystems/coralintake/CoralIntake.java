package frc.subsystems.coralintake;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.RuntimeConstants;
import frc.constants.Subsystems.CoralIntakeConstants;
import frc.subsystems.slapdown.GenericSlapdown;
import frc.subsystems.slapdown.GenericSlapdownCharacterizer;
import frc.subsystems.slapdown.GenericSlapdownIO;
import frc.subsystems.slapdown.GenericSlapdownIOInputsAutoLogged;
import frc.subsystems.slapdown.GenericSlapdownIOReal;
import frc.subsystems.slapdown.GenericSlapdownIOSim;
import frc.utils.Util;
import frc.utils.math.Algebra;
import frc.utils.math.DoubleDifferentiableValue;
import frc.utils.tuning.TuneableArmFF;
import frc.utils.tuning.TuneableProfiledPIDController;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends GenericSlapdown {
    private GenericSlapdownIO io;
    private GenericSlapdownIOInputsAutoLogged inputs = new GenericSlapdownIOInputsAutoLogged();
    private static CoralIntake instance;

    private TuneableArmFF ff = new TuneableArmFF(CoralIntakeConstants.kS, CoralIntakeConstants.kG,
        CoralIntakeConstants.kV, CoralIntakeConstants.kA, "CoralIntakeFF");
    @Getter private TuneableProfiledPIDController pid = new TuneableProfiledPIDController(CoralIntakeConstants.kP, 0,
        CoralIntakeConstants.kD,
        new Constraints(CoralIntakeConstants.BaseVelocityMax.get(), CoralIntakeConstants.BaseAccelerationMax.get()),
        "CoralIntakePID");

    @Getter private double goal = 0.0;
    @Getter private DoubleDifferentiableValue differentiableMeasurementDegrees = new DoubleDifferentiableValue();
    private double lastMeasurement = 0.0;
    @Getter private boolean intaking = false;
    @Getter private boolean outtaking = false;
    private boolean hasResetPidController = false;

    private CoralIntake() {
        io = (GenericSlapdownIO) Util.getIOImplementation(GenericSlapdownIOReal.class, GenericSlapdownIOSim.class,
            new GenericSlapdownIO() {});
        pid.setTolerance(CoralIntakeConstants.BaseGoalTolerance);
        io.configureHardware(CoralIntakeConstants.PivotMotorID, CoralIntakeConstants.IntakeMotorID,
            CoralIntakeConstants.ThroughborePort, CoralIntakeConstants.ThroughboreMin,
            CoralIntakeConstants.ThroughboreMax, -1);
        io.configureSim("", null, null, null, CoralIntakeConstants.RotorToArmRatio,
            CoralIntakeConstants.EffectiveLength, CoralIntakeConstants.MaxAngle.angle(),
            CoralIntakeConstants.GroundAngle.angle(), CoralIntakeConstants.ThroughboreMin,
            CoralIntakeConstants.ThroughboreMax, CoralIntakeConstants.Weight);

        if (RuntimeConstants.TuningMode) {
            GenericSlapdownCharacterizer.createInstance(this, "CoralIntake", CoralIntakeConstants.GroundAngle,
                CoralIntakeConstants.MaxAngle);
        }
    }

    public static CoralIntake getInstance() {
        if (instance == null) instance = new CoralIntake();
        return instance;
    }

    @Override
    public Command slapdown() {
        return runOnce(() -> {
            pid.setGoal(CoralIntakeConstants.GroundAngle.get());
        });
    }

    @Override
    public Command stow() {
        return runOnce(() -> {
            pid.setGoal(CoralIntakeConstants.StowAngle);
        });
    }

    public void setPivotGoal(double goal) {
        if (goal <= CoralIntakeConstants.MaxAngle.get() && goal >= CoralIntakeConstants.GroundAngle.get()) {
            this.goal = goal;
            pid.setGoal(goal);
        }
    }

    @Override
    public double getPivotAngle() { return lastMeasurement; }

    @Override
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

    @Override
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

    @Override
    public Command turnOffRollers() {
        return runOnce(() -> {
            io.setIntakePower(0);
            intaking = false;
            outtaking = false;
        });
    }

    public boolean pivotAtGoal() {
        return pid.atGoal();
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

    @Override
    public GenericSlapdownIO getIo() { return io; }
}
