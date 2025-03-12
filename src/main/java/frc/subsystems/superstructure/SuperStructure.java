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
import frc.utils.tuning.TuneableDistance;
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
    private double maxEEHeight = ElevatorConstants.MaxHeight.in(Inches) +
        EndEffectorConstants.EffectiveDistanceFromElevator.in(Inches);
    private double lastEEDistanceFromElevator = 0.0d;

    private double elevatorTilt = RobotMeasurements.ElevatorTilt.in(Radians);
    private double eeLength = EndEffectorConstants.EffectiveDistanceFromElevator.in(Inches);
    private double cIntakeLength = CoralIntakeConstants.EffectiveLength.in(Inches);

    private Angle ninetyDeg = Degrees.of(90); // Cache to avoid making more objects
    private Angle oneEightyDeg = Degrees.of(180);

    private TuneableNumber percentOfArmAccel = new TuneableNumber(0.2, "SuperStructure/percentOfArmAccel");
    private TuneableNumber percentOfArmVelo = new TuneableNumber(0.4, "SuperStructure/percentOfArmVelo");
    private TuneableBoolean usePredictiveWillCollide = new TuneableBoolean(false,
        "SuperStructure/usePredictiveWillCollide"); // Note: Sim tests indicate that this can cause jumpiness - worth testing IRL but not 100% necessary

    private TuneableNumber pctOfDriveAccelX = new TuneableNumber(0.2, "SuperStructure/pctOfDriveAccelX");
    private TuneableNumber pctOfDriveAccelY = new TuneableNumber(0.2, "SuperStructure/pctOfDriveAccelY");
    private TuneableNumber pctOfDriveAccelR = new TuneableNumber(0.35, "SuperStructure/pctOfDriveAccelR");

    private TuneableDistance algaeArmPivotElevatorHeight = new TuneableDistance(30,
        "SuperStructure/Min height arm can pivot at w/ algae");
    private TuneableDistance coralArmPivotElevatorHeight = new TuneableDistance(4,
        "SuperStructure/Min height that arm can leave cIntake"); // should be same as bump stow preset elevator height
    
    private TuneableBoolean stowCommandIsDefault = new TuneableBoolean(!RuntimeConstants.TuningMode, "SuperStructure/EnableStowAsDefault");

    private boolean freezingArmFromCoralContainmentDebounce = false;

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
                setDefaultCommand(beThereAsap(Presets.Stow));
            }
            stowCommandIsDefault.addListener((Boolean enable) -> {
                if (enable) setDefaultCommand(beThereAsap(Presets.Stow));
                else this.removeDefaultCommand();
            });
        } else {
            setDefaultCommand(beThereAsap(Presets.Stow));
        }
    }

    public static SuperStructure getInstance() {
        if (instance == null) instance = new SuperStructure();
        return instance;
    }

    public void setArmGoalSafely(Angle newGoal) {
        if (stateIsValid(elevator.getGoal(), newGoal, coralIntake.getGoal())) {
            arm.setGoal(newGoal);
        }
    }

    public void setElevatorGoalSafely(Distance newGoal) {
        if (stateIsValid(newGoal, arm.getGoal(), coralIntake.getGoal())) {
            elevator.setGoal(newGoal);
        }
    }

    /**
     * Changes arm goal by changeBy. Accounts for collision detection. Do NOT require the arm or superstructure if you use this in an instantcommand.
     */
    public void changeArmGoalSafely(Angle changeBy) {
        setArmGoalSafely(arm.getGoal().plus(changeBy));
    }

    /**
     * Changes elevator goal by changeBy. Accounts for collision detection. Do NOT require the arm or superstructure if you use this in an instantcommand.
     */
    public void changeElevatorGoalSafely(Distance changeBy) {
        setElevatorGoalSafely(elevator.getGoal().plus(changeBy));
    }

    /**
     * Immediately sets all superstructure mechanism goals, and moves all mechanisms such that the risk of tipping the drivebase is minimized (ie. elevator moves last). This command does not finish, meaning the superstructure will stay here until you start a new command or interrupt this one. If you are using this in a composition and want to see when it finishes, use ::allAtGoal.
     */
    public Command beThereIn(double secondsToBeThereIn, TuneableSuperStructureState goal) {
        Timer timeTaken = new Timer();
        Command command = runOnce(() -> {
            timeTaken.stop();
            timeTaken.reset();

            elevator.setGoal(goal.getHeight().distance());
            arm.setGoal(goal.getArm().angle());
            coralIntake.setPivotGoal(goal.getCoralIntake().angle());

            armSwitchingToFrontSide = arm.getMeasurement().gte(ninetyDeg) && arm.getGoal().lt(ninetyDeg);
            armSwitchingToBackSide = arm.getMeasurement().lte(ninetyDeg) && arm.getGoal().gt(ninetyDeg);

            lastElevatorTimePrediction = elevator.getPid().timeToGetTo(goal.getHeight().in(Inches),
                elevator.getMeasurement().in(Inches));
            isFreezingArm = false;
            timeTaken.start();
            System.out.println("Superstructure beThereIn - front switch: " + armSwitchingToFrontSide + ", back switch: "
                + armSwitchingToBackSide + ", elevator time prediction: " + lastElevatorTimePrediction);
        }).andThen(run(() -> {
            boolean freezeArmFromAlgaeContainmentElevation = false;
            boolean freezeArmFromCoralContainment = false;
            boolean armOnTargetSide = true;

            if (!elevator.atGoal()) {
                if (elevator.getPid().goalError() < 0) {
                    if (!armWillCollideWithDrivebase) {
                        // If elevator wants to move down and it won't collide the arm with the drivebase, then move
                        elevator.getPid().setConstraints(
                            new Constraints(ElevatorConstants.BaseVelocityMax, ElevatorConstants.BaseAccelerationMax));
                        System.out.println("elevator wants to move down and can");
                    } else {
                        // Stop elevator so the arm can get out of the way
                        elevator.getPid().setConstraints(new Constraints(0, ElevatorConstants.BaseAccelerationMax));
                        System.out.println("elevator wants to move down and CAN'T");
                    }
                } else {
                    boolean elevatorCanMove = lastElevatorTimePrediction > (secondsToBeThereIn - timeTaken.get());
                    if (armSwitchingToFrontSide || armSwitchingToBackSide) {
                        armOnTargetSide = (armSwitchingToFrontSide && arm.getMeasurement().lt(ninetyDeg))
                            || (armSwitchingToBackSide && arm.getMeasurement().gt(ninetyDeg));
                    }
                    freezeArmFromCoralContainment = !arm.atGoal() && arm.getMeasurement().gte(oneEightyDeg)
                        && elevator.getMeasurement().lte(coralArmPivotElevatorHeight.distance())
                        && EndEffector.getInstance().hasCoral();
                    if (freezeArmFromCoralContainment) System.out.println("freezeArmFromCoralContainment");
                    if (elevatorCanMove && (armOnTargetSide || freezeArmFromCoralContainment)) {
                        // if elevator wants to move up, it's time for it to move up AND the arm is mostly done getting to its goal, then the elevator can move
                        elevator.getPid().setConstraints(
                            new Constraints(ElevatorConstants.BaseVelocityMax, ElevatorConstants.BaseAccelerationMax));
                    } else {
                        elevator.getPid().setConstraints(new Constraints(0, ElevatorConstants.BaseAccelerationMax));
                    }

                    // if we have an algae, we have to switch sides AND the elevator is too low
                    if (EndEffector.getInstance().hasAlgae() && !armOnTargetSide
                        && elevator.getMeasurement().gt(algaeArmPivotElevatorHeight.distance())) {
                        freezeArmFromAlgaeContainmentElevation = true;
                    } else {
                        freezeArmFromAlgaeContainmentElevation = false;
                    }
                }
            }

            if (!coralIntake.pivotAtGoal()) {
                if (coralIntake.getPid().goalError() > 0) {
                    if (armWillCollideWithCoralIntake || arm.getMeasurement().gte(Degrees.of(135))) {
                        coralIntake.getPid()
                            .setConstraints(new Constraints(0, CoralIntakeConstants.BaseAccelerationMax));
                    } else {
                        coralIntake.getPid().setConstraints(new Constraints(CoralIntakeConstants.BaseVelocityMax,
                            CoralIntakeConstants.BaseAccelerationMax));
                    }
                } else { // if it wants to go down, we need to make sure the arm wont hit it
                    if (armWillCollideWithCoralIntake) {
                        isFreezingArm = true;
                    } else {
                        isFreezingArm = false;
                    }
                }
            } else {
                isFreezingArm = false;
            }

            if (freezeArmFromAlgaeContainmentElevation) isFreezingArm = true;
            if (freezeArmFromCoralContainment && !freezingArmFromCoralContainmentDebounce) {
                isFreezingArm = true;
                freezingArmFromCoralContainmentDebounce = true;
            }
            if (!freezeArmFromCoralContainment && freezingArmFromCoralContainmentDebounce) {
                isFreezingArm = false;
                freezingArmFromCoralContainmentDebounce = false;
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
            // double elevatorFreezeTime = elevator.getPid().timeToStop();
            // double armFreezeTime = arm.getPid().timeToStop();
            // double coralFreezeTime = coralIntake.getPid().timeToStop();
            elevatorHeight = Inches.of(elevator.getDifferentiableMeasurementInches().firstDerivativeLinearApprox(0.04));
            armAngle = Degrees.of(arm.getDifferentiableMeasurementDegrees().firstDerivativeLinearApprox(0.04));
            cIntakeAngle = Degrees
                .of(coralIntake.getDifferentiableMeasurementDegrees().firstDerivativeLinearApprox(0.04));
        } else {
            elevatorHeight = elevator.getMeasurement();
            armAngle = arm.getMeasurement();
            cIntakeAngle = coralIntake.getPivotAngle();
        }

        boolean[] currentCollisions = collisionTest(elevatorHeight, armAngle, cIntakeAngle, true);
        armWillCollideWithDrivebase = currentCollisions[0];
        armWillCollideWithCoralIntake = currentCollisions[1];
    }

    private boolean[] collisionTest(Distance elevator, Angle arm, Angle cIntake, boolean updateCurrentMeasures) {
        boolean[] collisions = new boolean[2];

        double height = elevator.in(Inches);
        Point2d carriagePoint = new Point2d(4.5 - height * Trig.sizzle(elevatorTilt),
            height * Trig.cosizzle(elevatorTilt) + 1.64);
        double armAngle = -arm.in(Radians) + Units.degreesToRadians(90) - elevatorTilt;
        Point2d endEffector = new Point2d(carriagePoint.x() - Trig.sizzle(armAngle) * eeLength,
            carriagePoint.y() + Trig.cosizzle(armAngle) * eeLength);
        // System.out.println(" endEffector: " + endEffector);

        if (arm.gt(ninetyDeg)) { // Test for coral intake
            Point2d eeBottomTip = new Point2d(endEffector.x() + Trig.cosizzle(armAngle) * 3,
                endEffector.y() + Trig.sizzle(armAngle) * 3);
            collisions[0] = eeBottomTip.x() < 18 && eeBottomTip.y() < 0;

            Point2d cIntakeEnd = new Point2d(10.4 + Trig.cosizzle(cIntake) * cIntakeLength,
                0.5 + Trig.sizzle(cIntake) * cIntakeLength);

            double lineResult = ((eeBottomTip.y() - carriagePoint.y()) / (eeBottomTip.x() - carriagePoint.x())) *
                (cIntakeEnd.x() - carriagePoint.x()) + carriagePoint.y();
            // System.out.println(lineResult + " is less than " + cIntakeEnd.y());
            // System.out.println(" eeBottomTip: " + eeBottomTip);
            // System.out.println(" carriagePoint: " + carriagePoint);
            collisions[1] = lineResult < cIntakeEnd.y() && eeBottomTip.x() > cIntakeEnd.x();
        } else {
            Point2d eeTopTip = new Point2d(
                carriagePoint.x() + Trig.sizzle(armAngle) * 13 - 15.5 * Trig.cosizzle(armAngle + elevatorTilt),
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

    public boolean stateIsValid(Distance elevator, Angle arm, Angle cIntake) {
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
