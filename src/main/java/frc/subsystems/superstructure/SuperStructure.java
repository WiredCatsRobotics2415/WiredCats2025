package frc.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.constants.RuntimeConstants;
import frc.constants.Subsystems.EndEffectorConstants;
import frc.robot.Robot;
import frc.subsystems.arm.Arm;
import frc.subsystems.elevator.Elevator;
import frc.utils.math.Trig;
import frc.utils.tuning.TuningModeTab;
import lombok.Getter;

/**
 * Helper class to sit between commands and the arm and elevator, to ensure no internal collisions happen. All commands that use the arm or elevator should interact with this class instead of directly with arm or elevator.
 */
public class SuperStructure {
    private Arm arm = Arm.getInstance();
    private Elevator elevator = Elevator.getInstance();

    @Getter private boolean collisionPrevented = true;

    private static SuperStructure instance;

    private SuperStructure() {
        if (RuntimeConstants.TuningMode)
            TuningModeTab.getInstance().addBoolSupplier("Collision Prevented", this::isCollisionPrevented);
    }

    public static SuperStructure getInstance() {
        if (instance == null) instance = new SuperStructure();
        return instance;
    }

    /** Change arm goal by changeBy. Negatives work, bounds are checked. Intended for manual control. */
    public Command changeArmGoalBy(Angle changeBy) {
        return new RepeatCommand(new InstantCommand(() -> {
            this.setArmGoalSafely((arm.getGoal().plus(changeBy)));
        }, arm));
    }

    /** Change elevator goal by changeBy. Negatives work, bounds are checked. Intended for manual control. */
    public Command changeElevatorGoalBy(Distance changeBy) {
        return new RepeatCommand(new InstantCommand(() -> {
            this.setElevatorGoalSafely((elevator.getGoal().plus(changeBy)));
        }, elevator));
    }

    /**
     * Sets goals and waits for both mechanisms to achieve them. This method ensures that, while moving, there will be no collisions
     */
    public Command runToPositionCommand(Distance elevatorGoal, Angle armGoal) {
        return new RepeatCommand(new InstantCommand(() -> {
            arm.setGoal(armGoal);
            double elevatorHeightNextTimestep = elevator.getDifferentiableMeasurementInches()
                .firstDerivativeLinearApprox(0.02);
            double armAngleNextTimestep = arm.getDifferentiableMeasurementDegrees().firstDerivativeLinearApprox(0.02);
            if (positionsWillCollide(Inches.of(elevatorHeightNextTimestep), Degrees.of(armAngleNextTimestep))) {
                elevator.setGoal(elevator.getMeasurement());
            } else {
                elevator.setGoal(elevatorGoal);
            }
        }, elevator, arm)).until(this::bothAtGoal);
    }

    public void setArmGoalSafely(Angle armGoal) {
        if (!positionsWillCollide(elevator.getGoal(), armGoal)) {
            arm.setGoal(armGoal);
        }
    }

    public void setElevatorGoalSafely(Distance elevatorGoal) {
        if (!positionsWillCollide(elevatorGoal, arm.getGoal())) {
            elevator.setGoal(elevatorGoal);
        }
    }

    public void goToPosition(Distance elevatorGoal, Angle armGoal) {
        if (!positionsWillCollide(elevatorGoal, armGoal)) {
            elevator.setGoal(elevatorGoal);
            arm.setGoal(armGoal);
        }
    }

    // TODO: add in PID values for elevator and arm at goal (not working in simulator)
    public boolean bothAtGoal() {
        if (Robot.isSimulation()) {
            return true;
        }
        return elevator.atGoal() && arm.atGoal();
    }

    public boolean positionsWillCollide(Distance elevatorHeight, Angle armAngle) {
        boolean willCollide = (elevatorHeight.in(Inches) +
            Trig.sizzle(armAngle) * (EndEffectorConstants.EffectiveDistanceFromElevator.in(Inches)) < 0);
        this.collisionPrevented = willCollide;
        return willCollide;
    }
}
