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
import frc.subsystems.coralintake.CoralIntake;
import frc.subsystems.elevator.Elevator;
import frc.utils.math.Trig;
import frc.utils.tuning.TuneableBoolean;
import frc.utils.tuning.TuneableNumber;
import frc.utils.tuning.TuningModeTab;
import lombok.Getter;
import frc.subsystems.superstructure.TuneableSuperStructureState;

/**
 * Helper class to sit between commands and the arm and elevator, to ensure no internal collisions happen. All commands that use the arm or elevator should interact with this class instead of directly with arm or elevator.
 */
public class SuperStructure extends SubsystemBase {
    private Arm arm = Arm.getInstance();
    private CoralIntake intake = CoralIntake.getInstance();
    private Elevator elevator = Elevator.getInstance();
    private CoralIntake coralIntake = CoralIntake.getInstance();

    @Getter private boolean collisionPrevented = false;
    private double lastElevatorTimePrediction;
    private double lastArmStartError;

    private TuneableNumber percentageOfArmGoal = new TuneableNumber(0.0, "SuperStructure/percentageOfArmGoal"); //Note: Sim tests indicate this number should be at 0, otherwise we get wierd elevator freezing behavior - possibly add debounce so once arm has reached x% of goal, elevator can't be frozen again
    private TuneableNumber percentOfArmAccel = new TuneableNumber(0.2, "SuperStructure/percentOfArmAccel");
    private TuneableNumber percentOfArmVelo = new TuneableNumber(0.4, "SuperStructure/percentOfArmVelo");
    private TuneableBoolean usePredictiveWillCollide = new TuneableBoolean(false, "SuperStructure/usePredictiveWillCollide");

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
        if (!positionsWillCollide(elevator.getGoal(), armGoal, intake.getGoal())) {
            arm.setGoal(armGoal);
        }
    }

    public void setElevatorGoalSafely(Distance elevatorGoal) {
        if (!positionsWillCollide(elevatorGoal, arm.getGoal(), intake.getGoal())) {
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
            lastArmStartError = Math.abs(arm.getPid().getPositionError());
            timeTaken.start();
        }).andThen(run(() -> {
            if (!elevator.atGoal()) {
                if (elevator.getPid().getPositionError() > 0) {
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
                    boolean armHasReachedMostOfGoal = (Math.abs(arm.getPid().getPositionError()) /
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
        })).until(this::allAtGoal).andThen(runOnce(() -> {
            timeTaken.stop();
            timeTaken.reset();
        }));
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
        if (usePredictiveWillCollide.get()) {
            double elevatorFreezeTime = elevator.getPid().timeToStop();
        double armFreezeTime = arm.getPid().timeToStop();
        return positionsWillCollide(
        Inches.of(elevator.getDifferentiableMeasurementInches().firstDerivativeLinearApprox(elevatorFreezeTime)),
        Degrees.of(arm.getDifferentiableMeasurementDegrees().firstDerivativeLinearApprox(armFreezeTime)), Degrees.of(coralIntake.getDifferentiableMeasurementDegrees().firstDerivativeLinearApprox(coralfree)));
        } else {
            return positionsWillCollide(elevator.getMeasurement(), arm.getMeasurement(), coralIntake.getPivotAngle());
        }
    }

    private boolean positionsWillCollide(Distance elevatorHeight, Angle armAngle, Angle cintakeAngle) {
        Distance ElevatorY = elevatorHeight.times(Trig.cosizzle(3.7)).plus(Inches.of(1.64));
        Distance Army = ElevatorY.plus(ArmConstants.EffectiveLength.times(Trig.cosizzle(-armAngle.in(Degrees) - 3.7)));
        Distance Tipy = Army.plus(elevatorHeight.times(Trig.sizzle(armAngle)));
        boolean willCollideCintake = false;

        boolean willCollideElevator = ((elevatorHeight.in(Inches) +
            Trig.cosizzle(armAngle) * (EndEffectorConstants.EffectiveDistanceFromElevator.in(Inches))) < 0);
        if (cintakeAngle.in(Degrees) > 0) {
            willCollideCintake = (Tipy.minus(Inches.of(0.5)).in(Inches) < 0);
        } else if (cintakeAngle.in(Degrees) < 0) {
            willCollideCintake = ((Tipy.minus(Inches.of((0.5 * Trig.sizzle(cintakeAngle)))).in(Inches)) < 0);
        }
        this.collisionPrevented = willCollideElevator && willCollideCintake;
        return willCollideElevator && willCollideCintake;
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
