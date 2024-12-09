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
import frc.util.Utils;
import frc.util.driver.DashboardManager;
import frc.util.driver.DashboardManager.LayoutConstants;
import frc.util.visualization.RobotVisualizer;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private ArmFeedforward ff = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG,
        ArmConstants.kV, ArmConstants.kA);
    private ProfiledPIDController pid = new ProfiledPIDController(ArmConstants.kP, 0.0d,
        ArmConstants.kD,
        new TrapezoidProfile.Constraints(ArmConstants.veloMax, ArmConstants.accelMax));

    @Getter private double goalInDegrees = ArmConstants.potentiometerMinAngle;

    private boolean isCoasting = false;

    private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private static Arm instance;

    private Arm() {
        io = (ArmIO) Utils.getIOImplementation(ArmIOPotentiometer.class, ArmIOSim.class,
            ArmIO.class);

        DashboardManager.getInstance().addCommand(true, "Coast or Brake",
            new InstantCommand(() ->
            {
                if (isCoasting) {
                    coast();
                } else {
                    brake();
                }
            }, this), LayoutConstants.CoastCommand);
        DashboardManager.getInstance().addBoolSupplier(true, "Limit Switch",
            () -> inputs.limitSwitch, LayoutConstants.LimitSwitch);
    }

    public static Arm getInstance() {
        if (instance == null) instance = new Arm();
        return instance;
    }

    /**
     * Sets the left arm's motor to the desired voltage, calculated by the feedforward
     * object and PID subsystem.
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

    /**
     * Sets the goal of the arm in degrees. Does NOT account for the goal being a value
     * outside of the minimum or maximum degrees.
     *
     * @param goalInRotations
     */
    public void setGoal(double goalInDegrees) {
        this.goalInDegrees = goalInDegrees;
        pid.setGoal(new TrapezoidProfile.State(goalInDegrees, 0));
    }

    /**
     * @return A command to increase the arm's current goal by one degree. Does not go above
     *         max rotations defined in constants.
     */
    public Command increaseGoal() {
        return new RepeatCommand(new InstantCommand(() -> {
            if (goalInDegrees >= ArmConstants.potentiometerMaxAngle) {
                goalInDegrees = ArmConstants.potentiometerMaxAngle;
                return;
            }
            goalInDegrees += 0.5;
            this.setGoal(goalInDegrees);
        }));
    }

    /**
     * @return A command to decrease the arm's current goal by one degree. Does not go below
     *         min rotations defined in constants.
     */
    public Command decreaseGoal() {
        return new RepeatCommand(new InstantCommand(() -> {
            if (goalInDegrees <= ArmConstants.potentiometerMinAngle) {
                goalInDegrees = ArmConstants.potentiometerMinAngle;
                return;
            }
            goalInDegrees -= 0.5;
            this.setGoal(goalInDegrees);
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
        return MathUtil.isNear(goalInDegrees, inputs.position, ArmConstants.GoalTolerance);
    }

    /** Sets the min and max volt back to what they started as on boot from constants */
    public void resetPotentiometerAndArm() {
        io.setPotentiometerBounds(ArmConstants.potentiometerMinVolt,
            ArmConstants.potentiometerMaxVolt);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        useOutput(pid.calculate(inputs.position), pid.getSetpoint());

        RobotVisualizer.update(inputs.position, goalInDegrees);
    }
}
