package frc.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Subsystems.ArmConstants;
import frc.utils.Util;
import frc.utils.Visualizer;
import frc.utils.driver.DashboardManager;
import frc.utils.driver.DashboardManager.LayoutConstants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private ArmFeedforward ff = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);
    private ProfiledPIDController pid = new ProfiledPIDController(ArmConstants.kP, 0.0d, ArmConstants.kD,
        new TrapezoidProfile.Constraints(ArmConstants.veloMax, ArmConstants.accelMax));

    @Getter private double goalDegrees = 0.0;

    private boolean isCoasting = false;

    private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private static Arm instance;

    private Arm() {
        io = (ArmIO) Util.getIOImplementation(ArmIOReal.class, ArmIOSim.class, ArmIO.class);

        DashboardManager.getInstance().addCommand(true, "Coast or Brake", new InstantCommand(() -> {
            if (isCoasting) {
                coast();
            } else {
                brake();
            }
        }, this), LayoutConstants.CoastCommand);
    }

    public static Arm getInstance() {
        if (instance == null) instance = new Arm();
        return instance;
    }

    /**
     * Sets the left arm's motor to the desired voltage, calculated by the feedforward object and PID subsystem.
     *
     * @param output   the output of the ProfiledPIDController
     * @param setpoint the setpoint state of the ProfiledPIDController, for feedforward
     */
    private void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = ff.calculate(setpoint.position, setpoint.velocity);
        double voltOut = output + feedforward;
        if (!isCoasting) {
            io.setVoltage(voltOut);
        }
    }

    /** Sets the goal height. If goalInches is out of the physical range, it is not set. */
    public void setGoal(double goalDegrees) {
        if (goalDegrees > ArmConstants.MaxDegreesFront || goalDegrees < ArmConstants.MaxDegreesBack) return;
        this.goalDegrees = goalDegrees;
        pid.setGoal(new TrapezoidProfile.State(goalDegrees, 0));
    }

    public boolean atGoal() {
        return pid.atSetpoint();
    }

    /**
     * @return A command to increase the arm's current goal by one degree. Does not go above max rotations defined in constants.
     */
    public Command increaseGoal() {
        return new RepeatCommand(new InstantCommand(() -> {
            if (goalDegrees >= ArmConstants.MaxDegreesFront) {
                goalDegrees = ArmConstants.MaxDegreesFront;
                return;
            }
            goalDegrees += 0.5;
            this.setGoal(goalDegrees);
        }));
    }

    /**
     * @return A command to decrease the arm's current goal by one degree. Does not go below min rotations defined in constants.
     */
    public Command decreaseGoal() {
        return new RepeatCommand(new InstantCommand(() -> {
            if (goalDegrees <= ArmConstants.MaxDegreesBack) {
                goalDegrees = ArmConstants.MaxDegreesBack;
                return;
            }
            goalDegrees -= 0.5;
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

    public boolean withinSetGoalTolerance() {
        return MathUtil.isNear(goalDegrees, inputs.position, ArmConstants.GoalTolerance);
    }

    // Is this still needed?
    // public void resetPotentiometerAndArm() {
    // io.setPotentiometerBounds(ArmConstants.ThroughboreMin,
    // ArmConstants.ThroughboreMax);
    // }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        useOutput(pid.calculate(inputs.position), pid.getSetpoint());

        Visualizer.update();
    }
}
