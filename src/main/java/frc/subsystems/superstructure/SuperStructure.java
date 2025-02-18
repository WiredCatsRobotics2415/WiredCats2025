package frc.subsystems.superstructure;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.constants.RuntimeConstants;
import frc.constants.Subsystems.EndEffectorConstants;
import frc.subsystems.arm.Arm;
import frc.subsystems.elevator.Elevator;
import frc.utils.math.Algebra;
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
        }));
    }

    /** Change goal by changeBy. Negatives work, bounds are checked. Intended for manual control. */
    public Command changeElevatorGoalBy(Distance changeBy) {
        return new RepeatCommand(new InstantCommand(() -> {
            this.setElevatorGoalSafely((elevator.getGoal().plus(changeBy)));
        }));
    }

    /** Sets goals and waits for both mechanisms to achieve them */
    public Command runToPositionCommand(Distance elevatorGoal, Angle armGoal) {
        return new RepeatCommand(new InstantCommand(() -> {
            goToPosition(elevatorGoal, armGoal);
        })).until(this::bothAtGoal);
    }

    public void setArmGoalSafely(Angle armGoal) {
        if (!willCollide(elevator.getGoal(), armGoal)) {
            arm.setGoal(armGoal);
        }
    }

    public void setElevatorGoalSafely(Distance elevatorGoal) {
        if (!willCollide(elevatorGoal, arm.getGoal())) {
            elevator.setGoal(elevatorGoal);
        }
    }

    public void goToPosition(Distance elevatorGoal, Angle armGoal) {
        if (!willCollide(elevatorGoal, armGoal)) {
            elevator.setGoal(elevatorGoal);
            arm.setGoal(armGoal);
        }
    }

    public boolean bothAtGoal() {
        return elevator.atGoal() && arm.atGoal();
    }

    private boolean willCollide(Distance elevatorGoal, Angle armGoal) {
        boolean willCollide = (elevatorGoal.in(Inches) +
            Algebra.cosizzle(armGoal) * (EndEffectorConstants.EffectiveDistanceFromElevator.in(Inches)) < 0);
        this.collisionPrevented = willCollide;
        return willCollide;
    }
}
