package frc.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Controls.Presets;
import frc.constants.Measurements.RobotMeasurements;
import frc.constants.RuntimeConstants;
import frc.constants.Subsystems.ArmConstants;
import frc.constants.Subsystems.CoralIntakeConstants;
import frc.constants.Subsystems.ElevatorConstants;
import frc.constants.Subsystems.EndEffectorConstants;
import frc.subsystems.arm.Arm;
import frc.subsystems.coralintake.CoralIntake;
import frc.subsystems.elevator.Elevator;
import frc.utils.math.Point2d;
import frc.utils.math.Trig;
import frc.utils.tuning.TuneableBoolean;
import frc.utils.tuning.TuneableNumber;
import frc.utils.tuning.TuningModeTab;

/**
 * Helper class to sit between commands and the arm and elevator, to ensure no internal collisions happen. All commands that use the arm or elevator should interact with this class instead of directly with arm or elevator.
 */
public class SuperStructure extends SubsystemBase {
    private Arm arm = Arm.getInstance();
    private CoralIntake coralIntake = CoralIntake.getInstance();
    private Elevator elevator = Elevator.getInstance();

    private double lastElevatorTimePrediction;
    private double lastArmStartError;
    private boolean isFreezingArm = false;

    private boolean armWillCollideWithDrivebase = false;
    private boolean armWillCollideWithCoralIntake = false;

    private double elevatorTilt = RobotMeasurements.ElevatorTilt.in(Radians);
    private double eeLength = EndEffectorConstants.EffectiveDistanceFromElevator.in(Inches);
    private double cIntakeLength = CoralIntakeConstants.EffectiveLength.in(Inches);

    private TuneableNumber percentageOfArmGoal = new TuneableNumber(0.0, "SuperStructure/percentageOfArmGoal"); // Note: Sim tests indicate this number should be at 0, otherwise we get wierd elevator freezing behavior - possibly add debounce so once arm has reached x% of goal, elevator can't be frozen again. 3-8: the elevator completely freezes if you use this, so just don't. the arm moves fast enough here, it probably will IRL.
    private TuneableNumber percentOfArmAccel = new TuneableNumber(0.2, "SuperStructure/percentOfArmAccel");
    private TuneableNumber percentOfArmVelo = new TuneableNumber(0.4, "SuperStructure/percentOfArmVelo");
    private TuneableBoolean usePredictiveWillCollide = new TuneableBoolean(false,
        "SuperStructure/usePredictiveWillCollide"); // Note: Sim tests indicate that this can cause jumpiness - worth testing IRL but not 100% necessary

    private static SuperStructure instance;

    private SuperStructure() {
        if (RuntimeConstants.TuningMode) {
            TuningModeTab.getInstance().addBoolSupplier("Arm & Drivebase", () -> armWillCollideWithDrivebase);
            TuningModeTab.getInstance().addBoolSupplier("Arm & cIntake", () -> armWillCollideWithCoralIntake);
            TuningModeTab.getInstance().addBoolSupplier("freezing arm", () -> isFreezingArm);
        }

        setDefaultCommand(beThereAsap(Presets.Stow));
    }

    public static SuperStructure getInstance() {
        if (instance == null) instance = new SuperStructure();
        return instance;
    }

    /** Change arm goal by changeBy. Negatives work, bounds are checked. Intended for manual control. */
    public Command changeArmGoalBy(Angle changeBy) {
        return run(() -> {
            this.setArmGoalSafely((arm.getGoal().plus(changeBy)));
        }).finallyDo(() -> Commands.idle(this).schedule());
    }

    /** Change elevator goal by changeBy. Negatives work, bounds are checked. Intended for manual control. */
    public Command changeElevatorGoalBy(Distance changeBy) {
        return run(() -> {
            this.setElevatorGoalSafely((elevator.getGoal().plus(changeBy)));
        }).finallyDo(() -> Commands.idle(this).schedule());
    }

    public void setArmGoalSafely(Angle armGoal) {
        if (stateIsValid(elevator.getGoal(), armGoal, coralIntake.getGoal())) {
            arm.setGoal(armGoal);
        }
    }

    public void setElevatorGoalSafely(Distance elevatorGoal) {
        if (stateIsValid(elevatorGoal, arm.getGoal(), coralIntake.getGoal())) {
            elevator.setGoal(elevatorGoal);
        }
    }

    /**
     * Immediately sets all superstructure mechanism goals, and moves all mechanisms such that the risk of tipping the drivebase is minimized (ie. elevator moves last). This command does not finish, meaning the superstructure will stay here
     */
    public Command beThereIn(double secondsToBeThereIn, TuneableSuperStructureState goal) {
        Timer timeTaken = new Timer();
        Command command = runOnce(() -> {
            timeTaken.stop();
            timeTaken.reset();

            elevator.setGoal(goal.getHeight().distance());
            arm.setGoal(goal.getArm().angle());
            coralIntake.setPivotGoal(goal.getCoralIntake().angle());

            lastElevatorTimePrediction = elevator.getPid().timeToGetTo(goal.getHeight().in(Inches),
                elevator.getMeasurement().in(Inches));
            lastArmStartError = Math.abs(arm.getPid().goalError());
            isFreezingArm = false;
            timeTaken.start();
        }).andThen(run(() -> {
            if (!elevator.atGoal()) {
                if (elevator.getPid().goalError() < 0) {
                    if (!armWillCollideWithDrivebase) {
                        // If elevator wants to move down and it won't collide, then go
                        elevator.getPid().setConstraints(
                            new Constraints(ElevatorConstants.BaseVelocityMax, ElevatorConstants.BaseAccelerationMax));
                    } else {
                        // If elevator wants to move down and will collide, stop it so the arm can get out of the way
                        elevator.getPid().setConstraints(new Constraints(0, ElevatorConstants.BaseAccelerationMax));
                    }
                } else {
                    boolean elevatorCanMove = lastElevatorTimePrediction > (secondsToBeThereIn - timeTaken.get());
                    boolean armHasReachedMostOfGoal = (Math.abs(arm.getPid().goalError()) /
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
                if (coralIntake.getPid().goalError() > 0) {
                    if (armWillCollideWithCoralIntake) {
                        coralIntake.getPid()
                            .setConstraints(new Constraints(0, CoralIntakeConstants.BaseAccelerationMax));
                        System.out.println("cintake up frozen");
                    } else {
                        coralIntake.getPid().setConstraints(new Constraints(CoralIntakeConstants.BaseVelocityMax,
                            CoralIntakeConstants.BaseAccelerationMax));
                        System.out.println("cintake up moving");
                    }
                } else { // if it wants to go down, we need to make sure the arm wont hit it
                    if (armWillCollideWithCoralIntake) {
                        isFreezingArm = true;
                        System.out.println("cintake down");
                    } else {
                        isFreezingArm = false;
                        System.out.println("cintake down");
                    }
                }
            } else {
                isFreezingArm = false;
            }
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

    private void updateCurrentStateCollisions() {
        Distance elevatorHeight;
        Angle armAngle;
        Angle cIntakeAngle;
        if (usePredictiveWillCollide.get()) {
            double elevatorFreezeTime = elevator.getPid().timeToStop();
            double armFreezeTime = arm.getPid().timeToStop();
            double coralFreezeTime = coralIntake.getPid().timeToStop();
            elevatorHeight = Inches.of(elevator.getDifferentiableMeasurementInches().firstDerivativeLinearApprox(0.04));
            armAngle = Degrees.of(arm.getDifferentiableMeasurementDegrees().firstDerivativeLinearApprox(0.04));
            cIntakeAngle = Degrees
                .of(coralIntake.getDifferentiableMeasurementDegrees().firstDerivativeLinearApprox(0.04));
        } else {
            elevatorHeight = elevator.getMeasurement();
            armAngle = arm.getMeasurement();
            cIntakeAngle = coralIntake.getPivotAngle();
        }

        boolean[] currentCollisions = collisionTest(elevatorHeight, armAngle, cIntakeAngle);
        armWillCollideWithDrivebase = currentCollisions[0];
        armWillCollideWithCoralIntake = currentCollisions[1];
    }

    private boolean[] collisionTest(Distance elevator, Angle arm, Angle cIntake) {
        boolean[] collisions = new boolean[2];

        double height = elevator.in(Inches);
        Point2d carriagePoint = new Point2d(4.5 - height * Trig.sizzle(elevatorTilt),
            height * Trig.cosizzle(elevatorTilt) + 1.64);
        double armAngle = -arm.in(Radians) + Units.degreesToRadians(90) - elevatorTilt;
        Point2d endEffector = new Point2d(carriagePoint.getX() - Trig.sizzle(armAngle) * eeLength,
            carriagePoint.getY() + Trig.cosizzle(armAngle) * eeLength);
        // System.out.println(" endEffector: " + endEffector);

        if (arm.in(Degrees) > 90) { // Test for cintake
            Point2d eeBottomTip = new Point2d(endEffector.getX() + Trig.cosizzle(armAngle) * 3,
                endEffector.getY() + Trig.sizzle(armAngle) * 3);
            collisions[0] = eeBottomTip.getX() < 18 && eeBottomTip.getY() < 0;

            Point2d cIntakeEnd = new Point2d(10.4 + Trig.cosizzle(cIntake) * cIntakeLength,
                0.5 + Trig.sizzle(cIntake) * cIntakeLength);

            double lineResult = ((eeBottomTip.getY() - carriagePoint.getY()) /
                (eeBottomTip.getX() - carriagePoint.getX())) * (cIntakeEnd.getX() - carriagePoint.getX()) +
                carriagePoint.getY();
            // System.out.println(lineResult + " is less than " + cIntakeEnd.getY());
            // System.out.println(" eeBottomTip: " + eeBottomTip);
            // System.out.println(" carriagePoint: " + carriagePoint);
            collisions[1] = lineResult < cIntakeEnd.getY();
        } else {
            Point2d eeTopTip = new Point2d(
                carriagePoint.getX() + Trig.sizzle(armAngle) * 13 - 15.5 * Trig.cosizzle(armAngle + elevatorTilt),
                carriagePoint.getY() + Trig.cosizzle(armAngle) * 13 - 15.5 * Trig.sizzle(armAngle + elevatorTilt));
            collisions[0] = eeTopTip.getX() > -18 && eeTopTip.getY() < 0;

            collisions[1] = false;
        }
        return collisions;
    }

    public boolean stateIsValid(Distance elevator, Angle arm, Angle cIntake) {
        boolean[] collisions = collisionTest(elevator, arm, cIntake);
        return collisions[0] && collisions[1];
    }

    @Override
    public void periodic() {
        updateCurrentStateCollisions();

        if (!isFreezingArm) {
            // when elevator measurement is high, arm max accel should be % of its base max
            double maxArmAcceleration = ArmConstants.BaseAccelerationMax -
                ((elevator.getMeasurement().in(Inches) / ElevatorConstants.MaxHeight.in(Inches)) *
                    (1 - percentOfArmAccel.get()) * ArmConstants.BaseAccelerationMax);

            double maxArmVelocity = ArmConstants.BaseVelocityMax -
                (((Math.abs(elevator.getPid().goalError())) / ElevatorConstants.MaxHeight.in(Inches)) *
                    (1 - percentOfArmVelo.get()) * ArmConstants.BaseVelocityMax);
            arm.getPid().setConstraints(new Constraints(maxArmVelocity, maxArmAcceleration));
        } else {
            arm.getPid().setConstraints(new Constraints(0, ArmConstants.BaseAccelerationMax));
        }
    }
}
