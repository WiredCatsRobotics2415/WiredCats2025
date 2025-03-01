package frc.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.RuntimeConstants;
import frc.constants.Subsystems.ArmConstants;
import frc.subsystems.elevator.Elevator;
import frc.utils.Util;
import frc.utils.math.Algebra;
import frc.utils.math.DoubleDifferentiableValue;
import frc.utils.tuning.TuningModeTab;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Arm extends SubsystemBase {
    @Getter
    @AutoLogOutput(key = "Arm/Goal") private Angle goal = Degrees.of(0.0);
    @Getter private DoubleDifferentiableValue differentiableMeasurementDegrees = new DoubleDifferentiableValue();
    private boolean isCoasting = false;

    private ArmFeedforward ff = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);
    private ProfiledPIDController pid = new ProfiledPIDController(ArmConstants.kP, 0.0d, ArmConstants.kD,
        new TrapezoidProfile.Constraints(ArmConstants.VelocityMax, ArmConstants.AccelerationMax));

    @Getter private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private static Arm instance;

    private DoubleDifferentiableValue elevatorDDV = Elevator.getInstance().getDifferentiableMeasurementInches();
    private LoggedNetworkNumber elevatorVelocityMultiplier = new LoggedNetworkNumber("ArmVelocityMultiplier", 0);

    private Arm() {
        pid.setTolerance(ArmConstants.GoalTolerance.in(Degrees));
        io = (ArmIO) Util.getIOImplementation(ArmIOReal.class, ArmIOSim.class, new ArmIO() {});
        if (RuntimeConstants.TuningMode) {
            ArmCharacterization.enable(this);
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
        if (goal.gt(ArmConstants.MaxDegreesFront) || goal.lt(ArmConstants.MaxDegreesBack)) return;
        this.goal = goal;
        pid.setGoal(new TrapezoidProfile.State(goal.in(Radians), 0.0d));
    }

    /**
     * @return A command to increase the arm's current goal by one degree. Does not go above max rotations defined in constants.
     */
    public Command increaseGoal() {
        return runOnce(() -> {
            if (goal.gt(ArmConstants.MaxDegreesFront)) {
                goal = ArmConstants.MaxDegreesFront;
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
            if (goal.lt(ArmConstants.MaxDegreesBack)) {
                goal = ArmConstants.MaxDegreesBack;
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
        return pid.atSetpoint();
    }

    public Angle getMeasurement() {
        return Degrees.of(Algebra.linearMap(inputs.throughborePosition + ArmConstants.ThroughboreZero,
            ArmConstants.ThroughboreMin, ArmConstants.ThroughboreMax, ArmConstants.MaxDegreesBack.in(Degrees),
            ArmConstants.MaxDegreesFront.in(Degrees)));
    }

    private void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = ff.calculate(setpoint.position + (Math.PI / 2), setpoint.velocity);
        double voltOut = output + feedforward + elevatorVelocityMultiplier.get() * elevatorDDV.getFirstDerivative();
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
        useOutput(pid.calculate(measurementDegrees), pid.getSetpoint());

        Logger.recordOutput("Arm/Error", pid.getPositionError());
        Logger.recordOutput("Arm/ActualVelocity", differentiableMeasurementDegrees.getFirstDerivative());
        Logger.recordOutput("Arm/ActualAcceleration", differentiableMeasurementDegrees.getSecondDerivative());
    }
}
