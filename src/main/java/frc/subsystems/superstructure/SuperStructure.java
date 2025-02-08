package frc.subsystems.superstructure;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.constants.RuntimeConstants;
import frc.constants.Subsystems.ArmConstants;
import frc.constants.Subsystems.EndEffectorConstants;
import frc.subsystems.arm.Arm;
import frc.subsystems.elevator.Elevator;
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
    public Command changeArmGoalBy(Angle changeByDegrees) {
        return new RepeatCommand(new InstantCommand(() -> {
            this.setArmGoalSafely((arm.getGoalDegrees().plus(changeByDegrees)));
        }));
    }

    /** Change goal by changeBy. Negatives work, bounds are checked. Intended for manual control. */
    public Command changeElevatorGoalBy(Distance changeByInches) {
        return new RepeatCommand(new InstantCommand(() -> {
            this.setElevatorGoalSafely((elevator.getGoalInches().plus(changeByInches)));
        }));
    }

    /** Sets goals and waits for both mechanisms to achieve them */
    public Command runToPositionCommand(Distance elevatorGoalInches, Angle armGoalDegrees) {
        return new RepeatCommand(new InstantCommand(() -> {
            goToPosition(elevatorGoalInches, armGoalDegrees);
        })).until(this::bothAtGoal);
    }

    public void setArmGoalSafely(Angle armGoalDegrees) {
        if (!willCollide(elevator.getGoalInches(), armGoalDegrees)) {
            arm.setGoal(armGoalDegrees);
        }
    }

    public void setElevatorGoalSafely(Distance elevatorGoalInches) {
        if (!willCollide(elevatorGoalInches, arm.getGoalDegrees())) {
            elevator.setGoal(elevatorGoalInches);
        }
    }

    public void goToPosition(Distance elevatorGoalInches, Angle armGoalDegrees) {
        if (!willCollide(elevatorGoalInches, armGoalDegrees)) {
            elevator.setGoal(elevatorGoalInches);
            arm.setGoal(armGoalDegrees);
        }
    }

    public boolean bothAtGoal() {
        return elevator.atGoal() && arm.atGoal();
    }

    private boolean willCollide(Distance elevatorGoalInches, Angle armGoalDegrees) {
        boolean willCollide = (elevatorGoalInches.in(Inches) +
            Math.cos(armGoalDegrees.in(Radian)) * (ArmConstants.EffectiveLengthInches
                .plus(Inches.of(EndEffectorConstants.EffectiveLengthInches)).in(Inches)) < 0);
        this.collisionPrevented = willCollide;
        return willCollide;
    }
}
