package frc.subsystems.coralintake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.RuntimeConstants;
import frc.constants.Subsystems.CoralIntakeConstants;
import frc.subsystems.endeffector.EndEffector;
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
        new Constraints(CoralIntakeConstants.BaseVelocityMax, CoralIntakeConstants.BaseAccelerationMax),
        "CoralIntakePID");

    @Getter private Angle goal = Degrees.of(0.0);
    @Getter private DoubleDifferentiableValue differentiableMeasurementDegrees = new DoubleDifferentiableValue();
    @Getter private boolean intaking = false;
    @Getter private boolean outtaking = false;
    private boolean hasResetPidController = false;

    private CoralIntake() {
        io = (GenericSlapdownIO) Util.getIOImplementation(GenericSlapdownIOReal.class, GenericSlapdownIOSim.class,
            new GenericSlapdownIO() {});
        pid.setTolerance(CoralIntakeConstants.BaseGoalTolerance);
        io.configureHardware(CoralIntakeConstants.PivotMotorId, CoralIntakeConstants.IntakeMotorId,
            CoralIntakeConstants.ThroughborePort, -1);
        io.configureSim("", null, null, null, CoralIntakeConstants.RotorToArmRatio,
            CoralIntakeConstants.EffectiveLength, CoralIntakeConstants.MaxAngle, CoralIntakeConstants.GroundAngle,
            CoralIntakeConstants.ThroughboreMin, CoralIntakeConstants.ThroughboreMax, CoralIntakeConstants.Weight);

        new Trigger(EndEffector.getInstance()::hasCoral).onTrue(turnOffRollers());

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
            pid.setGoal(CoralIntakeConstants.GroundAngle.in(Degrees));
        });
    }

    @Override
    public Command stow() {
        return runOnce(() -> {
            pid.setGoal(CoralIntakeConstants.StowAngle.in(Degrees));
        });
    }

    public void setPivotGoal(Angle goal) {
        if (goal.lte(CoralIntakeConstants.MaxAngle) && goal.gte(CoralIntakeConstants.GroundAngle)) {
            this.goal = goal;
            pid.setGoal(goal.in(Degrees));
        }
    }

    @Override
    public Angle getPivotAngle() {
        return Degrees.of(Algebra.linearMap(inputs.throughborePosition + CoralIntakeConstants.ThroughboreZero,
            CoralIntakeConstants.ThroughboreMin, CoralIntakeConstants.ThroughboreMax,
            CoralIntakeConstants.MaxAngle.in(Degrees), CoralIntakeConstants.GroundAngle.in(Degrees)));
    }

    @Override
    public Command toggleIntake() {
        return runOnce(() -> {
            if (!intaking) {
                io.setIntakePower(CoralIntakeConstants.IntakeSpeed);
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
                io.setIntakePower(CoralIntakeConstants.OuttakeSpeed);
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
        return pid.atSetpoint();
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

        double measurementDegrees = getPivotAngle().in(Degrees);
        differentiableMeasurementDegrees.update(measurementDegrees);

        if (!hasResetPidController) {
            pid.reset(new TrapezoidProfile.State(measurementDegrees, 0));
            hasResetPidController = true;
        }
        useOutput(pid.calculate(measurementDegrees), pid.getSetpoint());
        Logger.recordOutput("CoralIntake/Goal", goal);
        Logger.recordOutput("CoralIntake/Error", pid.getPositionError());
    }

    @Override
    public GenericSlapdownIO getIo() { return io; }
}
