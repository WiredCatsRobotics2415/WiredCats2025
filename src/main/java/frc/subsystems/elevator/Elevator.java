package frc.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.RuntimeConstants;
import frc.constants.Subsystems.ElevatorConstants;
import frc.subsystems.arm.Arm;
import frc.subsystems.superstructure.SuperStructure;
import frc.utils.Util;
import frc.utils.math.DoubleDifferentiableValue;
import frc.utils.math.Trig;
import frc.utils.tuning.TuneableElevatorFF;
import frc.utils.tuning.TuneablePIDController;
import frc.utils.tuning.TuningModeTab;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private boolean coasting;
    @Getter private double goalInches;

    private double lastMeasurement = 0.0;
    @Getter private DoubleDifferentiableValue differentiableMeasurementInches = new DoubleDifferentiableValue();

    private TuneableElevatorFF ff = new TuneableElevatorFF(ElevatorConstants.kS, ElevatorConstants.kV,
        ElevatorConstants.kG, ElevatorConstants.kA, "ElevatorFF");
    @Getter private TuneablePIDController pid = new TuneablePIDController(ElevatorConstants.kP, 0.0d,
        ElevatorConstants.kD, "ElevatorPID");

    @Getter private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private static Elevator instance;

    private Elevator() {
        io = (ElevatorIO) Util.getIOImplementation(ElevatorIOReal.class, ElevatorIOSim.class, new ElevatorIO() {});
        pid.setTolerance(ElevatorConstants.BaseGoalTolerance);
        if (RuntimeConstants.TuningMode) {
            ElevatorCharacterization.enable(this);
            TuningModeTab.getInstance().addCommand("Run to 0",
                runOnce(() -> SuperStructure.getInstance().setElevatorGoalSafely(ElevatorConstants.MinHeight)));
            TuningModeTab.getInstance().addCommand("Run to max",
                runOnce(() -> SuperStructure.getInstance().setElevatorGoalSafely(ElevatorConstants.MaxHeight)));
            TuningModeTab.getInstance().addCommand("Toggle elevator coast", runOnce(() -> {
                if (coasting) {
                    io.setCoast(false);
                    coasting = false;
                } else {
                    io.setCoast(true);
                    coasting = true;
                }
            }));
        }
    }

    public static Elevator getInstance() {
        if (instance == null) instance = new Elevator();
        return instance;
    }

    /** Sets the goal height. If goalInches is out of the physical range, it is not set. */
    public void setGoal(double setGoal) {
        if (setGoal > ElevatorConstants.MaxHeight || setGoal < ElevatorConstants.MinHeight) return;
        this.goalInches = setGoal;
        pid.setSetpoint(setGoal);
    }

    public boolean atGoal() {
        return pid.atSetpoint();
    }

    public double getMeasurement() { return lastMeasurement; }

    private void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = ff.calculate(setpoint.velocity);

        double voltOut = output + feedforward + Trig.cosizzle(Arm.getInstance().getMeasurement()) *
            (ElevatorConstants.kGForArm.get() + ElevatorConstants.kGForArmWithAlgae.get());

        io.setVoltage(voltOut);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        differentiableMeasurementInches.update(inputs.inches);

        useOutput(pid.calculate(inputs.inches), new TrapezoidProfile.State(pid.getSetpoint(), 0));

        Logger.recordOutput("Elevator/Actual", inputs.inches);
        Logger.recordOutput("Elevator/Goal", goalInches);
        Logger.recordOutput("Elevator/Error", pid.getError());
        Logger.recordOutput("Elevator/ActualVelocity", differentiableMeasurementInches.getFirstDerivative());
        Logger.recordOutput("Elevator/ActualAcceleration", differentiableMeasurementInches.getSecondDerivative());
    }
}
