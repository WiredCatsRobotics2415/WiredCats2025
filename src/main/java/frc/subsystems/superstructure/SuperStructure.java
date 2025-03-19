package frc.subsystems.superstructure;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Controls.Presets;
import frc.constants.Measurements.RobotMeasurements;
import frc.constants.RuntimeConstants;
import frc.constants.Subsystems.ArmConstants;
import frc.constants.Subsystems.CoralIntakeConstants;
import frc.constants.Subsystems.DriveConstants;
import frc.constants.Subsystems.ElevatorConstants;
import frc.constants.Subsystems.EndEffectorConstants;
import frc.subsystems.arm.Arm;
import frc.subsystems.coralintake.CoralIntake;
import frc.subsystems.elevator.Elevator;
import frc.subsystems.endeffector.EndEffector;
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

    private boolean armSwitchingToFrontSide = false;
    private boolean armSwitchingToBackSide = false;

    private boolean armWillCollideWithDrivebase = false;
    private boolean armWillCollideWithCoralIntake = false;

    private double lastEEHeight = 0.0d;
    private double maxEEHeight = ElevatorConstants.MaxHeight +
        EndEffectorConstants.EffectiveDistanceFromElevator.in(Inches);
    private double lastEEDistanceFromElevator = 0.0d;

    private double elevatorTilt = RobotMeasurements.ElevatorTilt.in(Radians);
    private double eeLength = EndEffectorConstants.EffectiveDistanceFromElevator.in(Inches);
    private double cIntakeLength = CoralIntakeConstants.EffectiveLength.in(Inches);

    private TuneableNumber percentOfArmAccel = new TuneableNumber(0.2, "SuperStructure/percentOfArmAccel");
    private TuneableNumber percentOfArmVelo = new TuneableNumber(0.4, "SuperStructure/percentOfArmVelo");
    private TuneableBoolean usePredictiveWillCollide = new TuneableBoolean(false,
        "SuperStructure/usePredictiveWillCollide"); // Note: Sim tests indicate that this can cause jumpiness - worth testing IRL but not 100% necessary

    private TuneableNumber pctOfDriveAccelX = new TuneableNumber(0.2, "SuperStructure/pctOfDriveAccelX");
    private TuneableNumber pctOfDriveAccelY = new TuneableNumber(0.2, "SuperStructure/pctOfDriveAccelY");
    private TuneableNumber pctOfDriveAccelR = new TuneableNumber(0.35, "SuperStructure/pctOfDriveAccelR");

    private TuneableNumber swingThroughMinHeight = new TuneableNumber(35, "SuperStructure/swingThroughMinHeight");
    private TuneableNumber frontSideRightBeforeSwingThrough = new TuneableNumber(75,
        "SuperStructure/frontSideRightBeforeSwingThrough");
    private TuneableNumber backSideRightBeforeSwingThrough = new TuneableNumber(100,
        "SuperStructure/frontSideRightBeforeSwingThrough");
    private TuneableNumber rightBeforeHitCintake = new TuneableNumber(135, "SuperStructure/rightBeforeHitCintake");
    private TuneableNumber hasCoralMinHeightBeforeSwing = new TuneableNumber(135,
        "SuperStructure/hasCoralMinHeightBeforeSwing");
    private TuneableBoolean considerCintake = new TuneableBoolean(false, "SuperStructure/considerCintake");

    private Point2d carriagePoint = new Point2d(0, 0);
    private Point2d endEffector = new Point2d(0, 0);
    private Point2d eeBottomTip = new Point2d(0, 0);
    private Point2d cIntakeEnd = new Point2d(0, 0);
    private Point2d eeTopTip = new Point2d(0, 0);

    private boolean isDone = true;
    private TuneableSuperStructureState goal;

    private static SuperStructure instance;

    private SuperStructure() {
        if (RuntimeConstants.TuningMode) {
            TuningModeTab.getInstance().addBoolSupplier("Arm & Drivebase", () -> armWillCollideWithDrivebase);
            TuningModeTab.getInstance().addBoolSupplier("Arm & cIntake", () -> armWillCollideWithCoralIntake);
            TuningModeTab.getInstance().addBoolSupplier("Front Switch", () -> armSwitchingToFrontSide);
            TuningModeTab.getInstance().addBoolSupplier("Back Switch", () -> armSwitchingToBackSide);
            TuningModeTab.getInstance().addDoubleSupplier("last EE height", () -> lastEEHeight);
            TuningModeTab.getInstance().addDoubleSupplier("last EE distance", () -> lastEEDistanceFromElevator);
        }
        setDefaultCommand(stow());
    }

    public static SuperStructure getInstance() {
        if (instance == null) instance = new SuperStructure();
        return instance;
    }

    public Command stow() {
        return this.runOnce(() -> {
            if (EndEffector.getInstance().hasAlgae()) {
                beThereAsapNoEnd(Presets.AlgaeStow).schedule();
            } else {
                beThereAsapNoEnd(Presets.Stow).schedule();
            }
        });
    }

    public void setArmGoalSafely(double newGoal) {
        // if (stateIsValid(elevator.getGoalInches(), newGoal, coralIntake.getGoal())) {
        arm.setGoal(newGoal);
        // }
    }

    public void setElevatorGoalSafely(double newGoal) {
        // if (stateIsValid(newGoal, arm.getGoalDegrees(), coralIntake.getGoal())) {
        elevator.setGoal(newGoal);
        // }
    }

    /**
     * Changes arm goal by changeBy. Accounts for collision detection. Do NOT require the arm or superstructure if you use this in an instantcommand.
     */
    public void changeArmGoalSafely(double changeBy) {
        setArmGoalSafely(arm.getGoalDegrees() + changeBy);
    }

    /**
     * Changes elevator goal by changeBy. Accounts for collision detection. Do NOT require the arm or superstructure if you use this in an instantcommand.
     */
    public void changeElevatorGoalSafely(double changeBy) {
        setElevatorGoalSafely(elevator.getGoalInches() + changeBy);
    }

    /**
     * Depending on the side of the robot that the arm is on, set its goal to right before it would hit the igus chain
     */
    private void setArmGoalToRightBeforeSwingThrough() {
        if (arm.getMeasurement() <= 90) {
            arm.setGoal(frontSideRightBeforeSwingThrough.get());
        } else {
            arm.setGoal(backSideRightBeforeSwingThrough.get());
        }
    }

    private boolean armPastSwingThrough() {
        System.out.println("Arm switching to front? " + armSwitchingToFrontSide + " | Arm switching to back? "
            + armSwitchingToBackSide + " | m: " + arm.getMeasurement());
        return (armSwitchingToFrontSide && arm.getMeasurement() < 90)
            || (armSwitchingToBackSide && arm.getMeasurement() > 90);
    }

    /**
     * Returns a command that, when scheduled, schedules a command composition that moves the superstructure to the state. The composition requires all of the superstructure subsystems.
     */
    public Command beThereAsap(TuneableSuperStructureState goal) {
        return runOnce(() -> {
            Command plannedCommand;
            isDone = false;
            armSwitchingToFrontSide = arm.getMeasurement() > 90 && goal.getArm().get() < 90;
            armSwitchingToBackSide = arm.getMeasurement() < 90 && goal.getArm().get() > 90;
            boolean isMovingCoralIntake = considerCintake.get()
                ? !coralIntake.getPid().isHere(goal.getCoralIntake().get())
                : false;
            // if the arm does not need to switch sides, then we can just run it
            if (!armSwitchingToFrontSide && !armSwitchingToBackSide) {
                plannedCommand = Commands.runOnce(() -> {
                    elevator.setGoal(goal.getHeight().get());
                    arm.setGoal(goal.getArm().get());
                    coralIntake.setPivotGoal(goal.getCoralIntake().get());
                });
            } else {
                // if the arm does have to switch sides
                // if the elevator has to move up, set its goal to the swingthrough height, set arm goal to right before swing through, once elevator is there, set arm and elevator goal to thier true values
                // if the elevator has to move down, set its goal to the swingthrough height and set arm goal to goal right before it would hit the cintake, then once the arm is past swingthrough then set elevator to its true goal
                elevator.setGoal(goal.getHeight().get()); // NOTE - this might not work, setting the goal then getting the error in the next call
                if (elevator.getPid().goalError() >= 0) {
                    System.out.println("Moving elevator UP to swingThroughMinHeight");
                    elevator.setGoal(swingThroughMinHeight.get());
                    setArmGoalToRightBeforeSwingThrough();
                    plannedCommand = Commands.waitUntil(() -> {
                        if (EndEffector.getInstance().hasCoral()
                            && elevator.getMeasurement() < hasCoralMinHeightBeforeSwing.get()) return false;
                        System.out.println("waiting for elevator to reach goal");
                        return elevator.atGoal();
                    }).andThen(Commands.runOnce(() -> {
                        elevator.setGoal(goal.getHeight().get());
                        arm.setGoal(goal.getArm().get());
                        if (isMovingCoralIntake) coralIntake.setPivotGoal(goal.getCoralIntake().get());
                    }));
                } else {
                    System.out.println("Moving elevator DOWN to swingThroughMinHeight");
                    elevator.setGoal(swingThroughMinHeight.get());
                    double armGoal = goal.getArm().get();
                    if (isMovingCoralIntake) {
                        arm.setGoal(armGoal > rightBeforeHitCintake.get() ? rightBeforeHitCintake.get() : armGoal);
                        coralIntake.setPivotGoal(goal.getCoralIntake().get());
                    } else {
                        arm.setGoal(armGoal);
                    }
                    plannedCommand = Commands.waitUntil(() -> {
                        if (!isMovingCoralIntake) {
                            System.out.println("NOT mving cintake");
                            return this.armPastSwingThrough();
                        } else {
                            return this.armPastSwingThrough() && coralIntake.pivotAtGoal();
                        }
                    }).andThen(Commands.runOnce(() -> {
                        arm.setGoal(armGoal);
                        elevator.setGoal(goal.getHeight().get());
                    }));
                }
            }
            plannedCommand = plannedCommand.andThen(Commands.waitUntil(this::allAtGoal))
                .andThen(Commands.runOnce(() -> isDone = true));
            plannedCommand.addRequirements(elevator, arm, coralIntake);
            plannedCommand.schedule();
        });
    }

    public Command beThereAsapNoEnd(TuneableSuperStructureState goal) {
        return beThereAsap(goal).andThen(Commands.idle(this));
    }

    public boolean doneWithMovement() {
        return isDone;
    }

    private boolean allAtGoal() {
        // TODO: add cintake
        return arm.atGoal() && elevator.atGoal();
    }

    private void updateCurrentStateCollisions() {
        double elevatorHeight;
        double armAngle;
        double cIntakeAngle;
        if (usePredictiveWillCollide.get()) {
            // double elevatorFreezeTime = elevator.getPid().timeToStop();
            // double armFreezeTime = arm.getPid().timeToStop();
            // double coralFreezeTime = coralIntake.getPid().timeToStop();
            elevatorHeight = elevator.getDifferentiableMeasurementInches().firstDerivativeLinearApprox(0.04);
            armAngle = arm.getDifferentiableMeasurementDegrees().firstDerivativeLinearApprox(0.04);
            cIntakeAngle = coralIntake.getDifferentiableMeasurementDegrees().firstDerivativeLinearApprox(0.04);
        } else {
            elevatorHeight = elevator.getMeasurement();
            armAngle = arm.getMeasurement();
            cIntakeAngle = coralIntake.getPivotAngle();
        }

        boolean[] currentCollisions = collisionTest(elevatorHeight, armAngle, cIntakeAngle, true);
        armWillCollideWithDrivebase = currentCollisions[0];
        armWillCollideWithCoralIntake = currentCollisions[1];
    }

    private boolean[] collisionTest(double elevator, double arm, double cIntake, boolean updateCurrentMeasures) {
        boolean[] collisions = new boolean[2];

        double armRadians = Units.degreesToRadians(arm);
        carriagePoint.set(4.5 - elevator * Trig.sizzle(elevatorTilt), elevator * Trig.cosizzle(elevatorTilt) + 1.64);
        double armAngle = -armRadians + Units.degreesToRadians(90) - elevatorTilt;
        endEffector.set(carriagePoint.x() - Trig.sizzle(armAngle) * eeLength,
            carriagePoint.y() + Trig.cosizzle(armAngle) * eeLength);
        // System.out.println(" endEffector: " + endEffector);

        if (arm > 90) { // Test for coral intake
            eeBottomTip.set(endEffector.x() + Trig.cosizzle(armAngle) * 3, endEffector.y() + Trig.sizzle(armAngle) * 3);
            collisions[0] = eeBottomTip.x() < 18 && eeBottomTip.y() < 0;

            // cIntakeEnd.set(10.4 + Trig.cosizzle(cIntake) * cIntakeLength, 0.5 + Trig.sizzle(cIntake) * cIntakeLength);

            // double lineResult = ((eeBottomTip.y() - carriagePoint.y()) / (eeBottomTip.x() - carriagePoint.x())) *
            // (cIntakeEnd.x() - carriagePoint.x()) + carriagePoint.y();
            // // System.out.println(lineResult + " is less than " + cIntakeEnd.y());
            // // System.out.println(" eeBottomTip: " + eeBottomTip);
            // // System.out.println(" carriagePoint: " + carriagePoint);
            // collisions[1] = lineResult < cIntakeEnd.y() && eeBottomTip.x() > cIntakeEnd.x();
            collisions[1] = false;
        } else {
            eeTopTip.set(carriagePoint.x() + Trig.sizzle(armAngle) * 13 - 15.5 * Trig.cosizzle(armAngle + elevatorTilt),
                carriagePoint.y() + Trig.cosizzle(armAngle) * 13 - 15.5 * Trig.sizzle(armAngle + elevatorTilt));
            collisions[0] = eeTopTip.x() > -18 && eeTopTip.y() < 0;

            collisions[1] = false;
        }
        if (updateCurrentMeasures) {
            lastEEHeight = endEffector.y();
            lastEEDistanceFromElevator = Math.abs(endEffector.x());
        }
        return collisions;
    }

    public boolean stateIsValid(double elevator, double arm, double cIntake) {
        boolean[] collisions = collisionTest(elevator, arm, cIntake, false);
        return !collisions[0] && !collisions[1];
    }

    public double[] recommendedDriveAccelLimits() {
        double[] limits = new double[3];
        limits[0] = DriveConstants.BaseXAccelerationMax.get() -
            (lastEEHeight / maxEEHeight) * (1 - pctOfDriveAccelX.get()) * DriveConstants.BaseXAccelerationMax.get();
        limits[1] = DriveConstants.BaseYAccelerationMax.get() -
            (lastEEHeight / maxEEHeight) * (1 - pctOfDriveAccelY.get()) * DriveConstants.BaseYAccelerationMax.get();
        limits[2] = DriveConstants.BaseRotationAccelMax.get() - (lastEEDistanceFromElevator / eeLength) *
            (1 - pctOfDriveAccelR.get()) * DriveConstants.BaseRotationAccelMax.get();
        return limits;
    }

    @Override
    public void periodic() {
        // updateCurrentStateCollisions();
        double currentMaxAccel = EndEffector.getInstance().hasAlgae() ? ArmConstants.AlgaeAccelerationMax.get()
            : ArmConstants.BaseAccelerationMax.get();
        double maxArmAcceleration = currentMaxAccel - ((elevator.getMeasurement() / ElevatorConstants.MaxHeight) *
            (1 - percentOfArmAccel.get()) * currentMaxAccel);

        double maxArmVelocity = ArmConstants.BaseVelocityMax.get() -
            (((Math.abs(elevator.getPid().goalError())) / ElevatorConstants.MaxHeight) * (1 - percentOfArmVelo.get()) *
                ArmConstants.BaseVelocityMax.get());

        arm.getPid().setConstraints(new Constraints(maxArmVelocity, maxArmAcceleration));
    }
}
