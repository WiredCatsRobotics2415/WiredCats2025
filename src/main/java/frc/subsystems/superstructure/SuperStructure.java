package frc.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Controls.Presets;
import frc.constants.RuntimeConstants;
import frc.constants.Subsystems.ArmConstants;
import frc.constants.Subsystems.CoralIntakeConstants;
import frc.constants.Subsystems.ElevatorConstants;
import frc.constants.Subsystems.EndEffectorConstants;
import frc.subsystems.arm.Arm;
import frc.subsystems.coralintake.CoralIntake;
import frc.subsystems.elevator.Elevator;
import frc.utils.math.Trig;
import frc.utils.tuning.TuneableNumber;
import frc.utils.tuning.TuningModeTab;
import lombok.Getter;

/**
 * Helper class to sit between commands and the arm and elevator, to ensure no internal collisions happen. All commands that use the arm or elevator should interact with this class instead of directly with arm or elevator.
 */
public class SuperStructure extends SubsystemBase {
    private Arm arm = Arm.getInstance();
    private Elevator elevator = Elevator.getInstance();
    private CoralIntake coralIntake = CoralIntake.getInstance();

    @Getter private boolean collisionPrevented = true;
    private double lastElevatorTimePrediction;
    private double lastArmStartError;

    private TuneableNumber percentageOfArmGoal = new TuneableNumber(0.75, "SuperStructure/percentageOfArmGoal");
    private TuneableNumber percentOfArmAccel = new TuneableNumber(0.2, "SuperStructure/percentOfArmAccel");
    private TuneableNumber percentOfArmVelo = new TuneableNumber(0.4, "SuperStructure/percentOfArmVelo");

    private static SuperStructure instance;

    private SuperStructure() {
        if (RuntimeConstants.TuningMode)
            TuningModeTab.getInstance().addBoolSupplier("Collision Prevented", this::isCollisionPrevented);

        setDefaultCommand(beThereAsap(Presets.Stow).repeatedly());
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

    /**
     * Immediately sets all superstructure mechanism goals, and moves all mechanisms such that the risk of tipping the drivebase is minimized (ie. elevator moves last).
     */
    public Command beThereIn(double secondsToBeThereIn, TuneableSuperStructureState goal) {
        Timer timeTaken = new Timer();
        Command command = runOnce(() -> {
            elevator.setGoal(goal.getHeight().distance());
            arm.setGoal(goal.getArm().angle());
            coralIntake.setPivotGoal(goal.getCoralIntake().angle());

            lastElevatorTimePrediction = elevator.getPid().timeToGetTo(goal.getHeight().in(Inches),
                elevator.getMeasurement().in(Inches));
            lastArmStartError = arm.getPid().getPositionError();
            timeTaken.start();
        }).andThen(run(() -> {
            if (!elevator.atGoal()) {
                if (elevator.getPid().getPositionError() < 0) {
                    if (!willCollideInNextTimestep()) {
                        // If elevator wants to move down and it won't collide, then go
                        elevator.getPid().setConstraints(
                            new Constraints(ElevatorConstants.BaseVelocityMax, ElevatorConstants.BaseAccelerationMax));
                    } else {
                        // If elevator wants to move down and will collide, stop it so the arm can get out of the way
                        elevator.getPid().setConstraints(new Constraints(0, ElevatorConstants.BaseAccelerationMax));
                    }
                } else {
                    boolean elevatorCanMove = lastElevatorTimePrediction > (secondsToBeThereIn - timeTaken.get());
                    boolean armHasReachedMostOfGoal = (arm.getPid().getPositionError() /
                        lastArmStartError) > percentageOfArmGoal.get();
                    if (elevatorCanMove && armHasReachedMostOfGoal) {
                        // if elevator wants to move up, it's time for it to move up AND the arm is mostly done getting to its goal, then the elevator can move
                        elevator.getPid().setConstraints(
                            new Constraints(ElevatorConstants.BaseVelocityMax, ElevatorConstants.BaseAccelerationMax));
                    } else {
                        elevator.getPid().setConstraints(new Constraints(0, ElevatorConstants.BaseAccelerationMax));
                    }
                }
            }

            if (!coralIntake.pivotAtGoal()) {
                if (coralIntake.getPid().getPositionError() > 0 && willCollideInNextTimestep()) {
                    coralIntake.getPid().setConstraints(new Constraints(0, CoralIntakeConstants.BaseAccelerationMax));
                } else {
                    coralIntake.getPid().setConstraints(new Constraints(CoralIntakeConstants.BaseVelocityMax,
                        CoralIntakeConstants.BaseAccelerationMax));
                }
            }
        })).until(this::allAtGoal);
        command.addRequirements(elevator, arm, coralIntake);
        return command;
    }

    public Command beThereAsap(TuneableSuperStructureState goal) {
        return beThereIn(0, goal);
    }

    public boolean allAtGoal() {
        return elevator.atGoal() && arm.atGoal() && coralIntake.pivotAtGoal();
    }

    private boolean willCollideInNextTimestep() {
        double elevatorFreezeTime = elevator.getPid().timeToStop();
        double armFreezeTime = arm.getPid().timeToStop();
        return positionsWillCollide(
            Inches.of(elevator.getDifferentiableMeasurementInches().firstDerivativeLinearApprox(elevatorFreezeTime)),
            Degrees.of(arm.getDifferentiableMeasurementDegrees().firstDerivativeLinearApprox(armFreezeTime)));
    }

    public boolean positionsWillCollide(Distance elevatorHeight, Angle armAngle) {
        boolean willCollide = (elevatorHeight.in(Inches) +
            Trig.sizzle(armAngle) * (EndEffectorConstants.EffectiveDistanceFromElevator.in(Inches)) < 0);
        this.collisionPrevented = willCollide;
        return willCollide;
    }

    @Override
    public void periodic() {
        // when elevator measurement is high, arm max accel should be % of its base max
        double maxArmAcceleration = ArmConstants.BaseAccelerationMax -
            ((elevator.getMeasurement().in(Inches) / ElevatorConstants.MaxHeight.in(Inches)) *
                (1 - percentOfArmAccel.get()) * ArmConstants.BaseAccelerationMax);
        double maxArmVelocity = ArmConstants.BaseVelocityMax -
            (((Math.abs(elevator.getPid().getPositionError())) / ElevatorConstants.MaxHeight.in(Inches)) *
                (1 - percentOfArmVelo.get()) * ArmConstants.BaseVelocityMax);
        arm.getPid().setConstraints(new Constraints(maxArmVelocity, maxArmAcceleration));
    }
}
