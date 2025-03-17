package frc.subsystems.superstructure;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
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

    private double lastElevatorTimePrediction;
    private boolean isFreezingArm = false;
    private boolean armSwitchingToFrontSide = false;
    private boolean armSwitchingToBackSide = false;

    private boolean armWillCollideWithDrivebase = false;
    private boolean armWillCollideWithCoralIntake = false;

    private double lastEEHeight = 0.0d;
    private double maxEEHeight = ElevatorConstants.MaxHeight +
        EndEffectorConstants.EffectiveDistanceFromElevator.in(Inches);
    private double lastEEDistanceFromElevator = 0.0d;

    private double elevatorTilt = Units.degreesToRadians(RobotMeasurements.ElevatorTilt);
    private double eeLength = EndEffectorConstants.EffectiveDistanceFromElevator.in(Inches);
    private double cIntakeLength = CoralIntakeConstants.EffectiveLength.in(Inches);

    private Timer timeTaken = new Timer();
    private double secondsToBeThereIn;

    private TuneableNumber percentOfArmAccel = new TuneableNumber(0.2, "SuperStructure/percentOfArmAccel");
    private TuneableNumber percentOfArmVelo = new TuneableNumber(0.4, "SuperStructure/percentOfArmVelo");
    private TuneableBoolean usePredictiveWillCollide = new TuneableBoolean(false,
        "SuperStructure/usePredictiveWillCollide"); // Note: Sim tests indicate that this can cause jumpiness - worth testing IRL but not 100% necessary

    private TuneableNumber pctOfDriveAccelX = new TuneableNumber(0.2, "SuperStructure/pctOfDriveAccelX");
    private TuneableNumber pctOfDriveAccelY = new TuneableNumber(0.2, "SuperStructure/pctOfDriveAccelY");
    private TuneableNumber pctOfDriveAccelR = new TuneableNumber(0.35, "SuperStructure/pctOfDriveAccelR");

    private TuneableNumber algaeArmPivotElevatorHeight = new TuneableNumber(40,
        "SuperStructure/Min height arm can pivot at w/ algae");
    private TuneableNumber coralArmPivotElevatorHeight = new TuneableNumber(4,
        "SuperStructure/Min height that arm can leave cIntake"); // should be same as bump stow preset elevator height
    private TuneableNumber swingThroughMinHeight = new TuneableNumber(34.75, "SuperStructure/swingThroughMinHeight");

    private TuneableBoolean stowCommandIsDefault = new TuneableBoolean(true, "SuperStructure/EnableStowAsDefault");

    private boolean freezingArmFromCoralContainmentDebounce = false;

    private Point2d carriagePoint = new Point2d(0, 0);
    private Point2d endEffector = new Point2d(0, 0);
    private Point2d eeBottomTip = new Point2d(0, 0);
    private Point2d cIntakeEnd = new Point2d(0, 0);
    private Point2d eeTopTip = new Point2d(0, 0);

    private Constraints armFreeze = new Constraints(0, ArmConstants.BaseAccelerationMax.get());

    private static SuperStructure instance;

    private SuperStructure() {
        if (RuntimeConstants.TuningMode) {
            TuningModeTab.getInstance().addBoolSupplier("Arm & Drivebase", () -> armWillCollideWithDrivebase);
            TuningModeTab.getInstance().addBoolSupplier("Arm & cIntake", () -> armWillCollideWithCoralIntake);
            TuningModeTab.getInstance().addBoolSupplier("Front Switch", () -> armSwitchingToFrontSide);
            TuningModeTab.getInstance().addBoolSupplier("Back Switch", () -> armSwitchingToBackSide);
            TuningModeTab.getInstance().addBoolSupplier("freezing arm", () -> isFreezingArm);
            TuningModeTab.getInstance().addDoubleSupplier("last EE height", () -> lastEEHeight);
            TuningModeTab.getInstance().addDoubleSupplier("last EE distance", () -> lastEEDistanceFromElevator);
            if (stowCommandIsDefault.get()) {
                setDefaultCommand(beThereAsapNoEnd(Presets.Stow));
            }
            stowCommandIsDefault.addListener((Boolean enable) -> {
                if (enable)
                    setDefaultCommand(beThereAsapNoEnd(Presets.Stow));
                else
                    this.removeDefaultCommand();
            });
        } else {
            setDefaultCommand(beThereAsapNoEnd(Presets.Stow));
        }
    }

    public static SuperStructure getInstance() {
        if (instance == null) instance = new SuperStructure();
        return instance;
    }

    public void setArmGoalSafely(double newGoal) {
        if (stateIsValid(elevator.getGoalInches(), newGoal, coralIntake.getGoal())) {
            arm.setGoal(newGoal);
        }
    }

    public void setElevatorGoalSafely(double newGoal) {
        if (stateIsValid(newGoal, arm.getGoalDegrees(), coralIntake.getGoal())) {
            elevator.setGoal(newGoal);
        }
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
     * Immediately sets all superstructure mechanism goals, and moves all mechanisms such that the risk of tipping the drivebase is minimized (ie. elevator moves last). This command is a "set it and forget it". if you want to wait until the superstructure is done: thisCommand.andThen(Commands.waitUntil(superstructure:allAtGoal))
     */
    public Command beThereIn(double secondsToBeThereIn, TuneableSuperStructureState goal) {
        Command command = runOnce(() -> {
            timeTaken.stop();
            timeTaken.reset();

            System.out.println("e goal: " + goal.getHeight().distance().in(Inches));
            elevator.setGoal(goal.getHeight().get());
            arm.setGoal(goal.getArm().get());
            coralIntake.setPivotGoal(goal.getCoralIntake().get());

            armSwitchingToFrontSide = arm.getMeasurement() > 90 && arm.getGoalDegrees() < 90;
            armSwitchingToBackSide = arm.getMeasurement() < 90 && arm.getGoalDegrees() > 90;

            // lastElevatorTimePrediction = elevator.getPid().timeToGetTo(goal.getHeight().get(),
            // elevator.getMeasurement().in(Inches));
            lastElevatorTimePrediction = 0;
            isFreezingArm = false;
            timeTaken.start();
            System.out.println("Superstructure beThereIn - front switch: " + armSwitchingToFrontSide + ", back switch: "
                + armSwitchingToBackSide + ", elevator time prediction: " + lastElevatorTimePrediction);
        });
        command.addRequirements(elevator, arm, coralIntake);
        return command;
    }

    public Command beThereAsap(TuneableSuperStructureState goal) {
        return beThereIn(0, goal).andThen(Commands.waitSeconds(0.04));
    }

    public Command beThereInNoEnd(double secondsToBeThereIn, TuneableSuperStructureState goal) {
        return beThereIn(secondsToBeThereIn, goal).andThen(Commands.idle(this));
    }

    public Command beThereAsapNoEnd(TuneableSuperStructureState goal) {
        return beThereAsap(goal).andThen(Commands.idle(this));
    }

    public boolean allAtGoal() {
        System.out.println("elevator: " + elevator.atGoal() + " | arm: " + arm.atGoal());
        return elevator.atGoal() && arm.atGoal();
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
        updateCurrentStateCollisions();

        boolean freezeArmFromAlgaeContainmentElevation = false;
        boolean armOnTargetSide = (arm.getGoalDegrees() < 90 && arm.getMeasurement() < 90)
            || (arm.getGoalDegrees() > 90 && arm.getMeasurement() > 90);

        if (!elevator.atGoal()) {
            if (elevator.getPid().goalError() < 0) {
                if (!armWillCollideWithDrivebase) {
                    // If elevator wants to move down and it won't collide the arm with the drivebase, then move
                    elevator.getPid().unfreeze();
                    // System.out.println("elevator wants to move down and can");
                } else {
                    // Stop elevator so the arm can get out of the way
                    elevator.getPid().freeze();;
                    // System.out.println("elevator wants to move down and CAN'T");
                }
            } else {
                boolean elevatorCanMove = lastElevatorTimePrediction > (secondsToBeThereIn - timeTaken.get());
                // if (armSwitchingToFrontSide || armSwitchingToBackSide) {
                // armOnTargetSide = (armSwitchingToFrontSide && arm.getMeasurement().lt(ninetyDeg))
                // || (armSwitchingToBackSide && arm.getMeasurement().gt(ninetyDeg));
                // }
                // freezeArmFromCoralContainment = !arm.atGoal() && arm.getMeasurement().gte(oneEightyDeg)
                // && elevator.getMeasurement().lte(coralArmPivotElevatorHeight.distance())
                // && EndEffector.getInstance().hasCoral();
                // if (freezeArmFromCoralContainment) System.out.println("freezeArmFromCoralContainment");
                // if (elevatorCanMove) {
                // // if elevator wants to move up, it's time for it to move up AND the arm is mostly done getting to its goal, then the elevator can move
                elevator.getPid().unfreeze();
                // } else {
                // elevator.getPid().setConstraints(new Constraints(0, ElevatorConstants.BaseAccelerationMax.get()));
                // }

                // if we have an algae, we have to switch sides AND the elevator is too low
                if (EndEffector.getInstance().hasAlgae() && !armOnTargetSide
                    && elevator.getMeasurement() < algaeArmPivotElevatorHeight.get()) {
                    freezeArmFromAlgaeContainmentElevation = true;
                } else {
                    freezeArmFromAlgaeContainmentElevation = false;
                }
            }
        }

        if (!armOnTargetSide && elevator.getMeasurement() < swingThroughMinHeight.get() && !elevator.atGoal()) {
            System.out.println("freezing arm for elevator to move");
            isFreezingArm = true;
        } else {
            isFreezingArm = false;
        }

        // if (!coralIntake.pivotAtGoal()) {
        // if (coralIntake.getPid().goalError() > 0) {
        // if (armWillCollideWithCoralIntake || arm.getMeasurement().gte(Degrees.of(135))) {
        // coralIntake.getPid()
        // .setConstraints(new Constraints(0, CoralIntakeConstants.BaseAccelerationMax.get()));
        // } else {
        // coralIntake.getPid().setConstraints(new Constraints(CoralIntakeConstants.BaseVelocityMax.get(),
        // CoralIntakeConstants.BaseAccelerationMax.get()));
        // }
        // } else { // if it wants to go down, we need to make sure the arm wont hit it
        // if (armWillCollideWithCoralIntake) {
        // isFreezingArm = true;
        // } else {
        // isFreezingArm = false;
        // }
        // }
        // } else {
        // isFreezingArm = false;
        // }

        if (freezeArmFromAlgaeContainmentElevation) isFreezingArm = true;

        if (!isFreezingArm) {
            // when elevator measurement is high, arm max accel should be % of its base max
            double maxArmAcceleration = ArmConstants.BaseAccelerationMax.get() -
                ((elevator.getMeasurement() / ElevatorConstants.MaxHeight) * (1 - percentOfArmAccel.get()) *
                    ArmConstants.BaseAccelerationMax.get());

            double maxArmVelocity = ArmConstants.BaseVelocityMax.get() -
                (((Math.abs(elevator.getPid().goalError())) / ElevatorConstants.MaxHeight) *
                    (1 - percentOfArmVelo.get()) * ArmConstants.BaseVelocityMax.get());
            arm.getPid().setConstraints(new Constraints(maxArmVelocity, maxArmAcceleration));
        } else {
            arm.getPid().setConstraints(armFreeze);
        }
    }
}
