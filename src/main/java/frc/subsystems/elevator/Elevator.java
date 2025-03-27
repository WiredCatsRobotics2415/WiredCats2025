package frc.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.RuntimeConstants;
import frc.constants.Subsystems.ElevatorConstants;
import frc.subsystems.arm.Arm;
import frc.subsystems.endeffector.EndEffector;
import frc.subsystems.superstructure.SuperStructure;
import frc.utils.Util;
import frc.utils.math.DoubleDifferentiableValue;
import frc.utils.math.Trig;
import frc.utils.tuning.TuneableNumber;
import frc.utils.tuning.TuneableProfiledPIDController;
import frc.utils.tuning.TuningModeTab;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private boolean coasting;
    @Getter private double goalInches;
    double voltOut;

    private double lastMeasurement = 0.0;
    @Getter private DoubleDifferentiableValue differentiableMeasurementInches = new DoubleDifferentiableValue();

    @Getter private TuneableProfiledPIDController pid = new TuneableProfiledPIDController(ElevatorConstants.kP, 0.0d,
        ElevatorConstants.kD, new Constraints(ElevatorConstants.BaseVelocityMax, ElevatorConstants.BaseVelocityMax),
        "ElevatorPID");

    private TuneableNumber noVoltDeadbandHeight = new TuneableNumber(1.5, "Elevator/noVoltDeadbandHeight");

    @Getter private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private static Elevator instance;

    private Elevator() {
        io = (ElevatorIO) Util.getIOImplementation(ElevatorIOReal.class, ElevatorIOSim.class, new ElevatorIO() {});
        pid.setTolerance(ElevatorConstants.BaseGoalTolerance);
        // if (Robot.isSimulation()) pid.setTolerance(ElevatorConstants.SimGoalTolerance);
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
        pid.setGoal(setGoal);
    }

    public boolean atGoal() {
        return pid.atGoal();
    }

    public double getMeasurement() { return lastMeasurement; }

    private void useOutput(double output, TrapezoidProfile.State setpoint) {
        if (getMeasurement() < noVoltDeadbandHeight.get() && pid.atGoal()) {
            io.setVoltage(0.0d);
            return;
        }

        double cosArm = Trig.cosizzle(Arm.getInstance().getMeasurement());
        if (getMeasurement() > 40) {
            voltOut = output + ((getMeasurement() - 40) / 38) * ElevatorConstants.MultConstant.get() +
                (cosArm * ElevatorConstants.kGForArm.get());
        } else {
            voltOut = output + ElevatorConstants.BottomConstant.get() + (cosArm * ElevatorConstants.kGForArm.get());;
        }
        if (EndEffector.getInstance().hasAlgae()) voltOut += ElevatorConstants.kGForArmWithAlgae.get();

        voltOut = MathUtil.clamp(voltOut, -1, 3);
        io.setVoltage(voltOut);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        lastMeasurement = inputs.inches;
        differentiableMeasurementInches.update(inputs.inches);

        useOutput(pid.calculate(inputs.inches), pid.getSetpoint());

        Logger.recordOutput("Elevator/Actual", inputs.inches);
        Logger.recordOutput("Elevator/Goal", goalInches);
        Logger.recordOutput("Elevator/Error", pid.goalError());
        Logger.recordOutput("Elevator/ActualVelocity", differentiableMeasurementInches.getFirstDerivative());
        Logger.recordOutput("Elevator/ActualAcceleration", differentiableMeasurementInches.getSecondDerivative());
    }
}
