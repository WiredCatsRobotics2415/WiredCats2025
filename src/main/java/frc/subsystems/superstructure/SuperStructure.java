package frc.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
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
    public Command changeArmGoalBy(double changeByDegrees) {
        return new RepeatCommand(new InstantCommand(() -> {
            this.setArmGoalSafely(arm.getGoalDegrees() + changeByDegrees);
        }));
    }

    /** Change goal by changeBy. Negatives work, bounds are checked. Intended for manual control. */
    public Command changeElevatorGoalBy(double changeByInches) {
        return new RepeatCommand(new InstantCommand(() -> {
            this.setElevatorGoalSafely(elevator.getGoalInches() + changeByInches);
        }));
    }

    /** Sets goals and waits for both mechanisms to achieve them */
    public Command runToPositionCommand(double elevatorGoalInches, double armGoalDegrees) {
        return new RepeatCommand(new InstantCommand(() -> {
            goToPosition(elevatorGoalInches, armGoalDegrees);
        })).until(this::bothAtGoal);
    }

    public void setArmGoalSafely(double armGoalDegrees) {
        if (!willCollide(elevator.getGoalInches(), armGoalDegrees)) {
            arm.setGoal(armGoalDegrees);
        }
    }

    public void setElevatorGoalSafely(double elevatorGoalInches) {
        if (!willCollide(elevatorGoalInches, arm.getGoalDegrees())) {
            elevator.setGoal(elevatorGoalInches);
        }
    }

    public void goToPosition(double elevatorGoalInches, double armGoalDegrees) {
        if (!willCollide(elevatorGoalInches, armGoalDegrees)) {
            elevator.setGoal(elevatorGoalInches);
            arm.setGoal(armGoalDegrees);
        }
    }

    public boolean bothAtGoal() {
        return elevator.atGoal() && arm.atGoal();
    }

    private boolean willCollide(double elevatorGoalInches, double armGoalDegrees) {
        boolean willCollide = elevatorGoalInches + Math.cos(Units.degreesToRadians(armGoalDegrees)) *
            (ArmConstants.EffectiveLengthInches + EndEffectorConstants.EffectiveLengthInches) < 0;
        this.collisionPrevented = willCollide;
        return willCollide;
    }
}
