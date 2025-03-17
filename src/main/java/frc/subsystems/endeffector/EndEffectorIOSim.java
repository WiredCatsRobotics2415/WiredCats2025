package frc.subsystems.endeffector;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.constants.Subsystems.EndEffectorConstants;
import frc.subsystems.arm.Arm;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.utils.Visualizer;
import frc.utils.math.Trig;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class EndEffectorIOSim implements EndEffectorIO {
    private DCMotorSim motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 1, 1),
        DCMotor.getNeo550(1));
    private final Arm armSubsystem = Arm.getInstance();

    private final IntakeSimulation intakeSimulation;

    public EndEffectorIOSim() {
        intakeSimulation = IntakeSimulation.OverTheBumperIntake("Coral",
            CommandSwerveDrivetrain.getInstance().getMapleSimSwerveDrivetrain().mapleSimDrive, Inches.of(9.5),
            Inches.of(11.3), IntakeSide.BACK, 1);
    }

    private void outtakeNote() {
        Pose3d carriagePose = Visualizer.getLastCarriagePoint();
        Angle armAngle = Degrees.of(armSubsystem.getMeasurement());
        Angle endEffectorAngle = armAngle.plus(EndEffectorConstants.AngleFromArmWrtCarraige);

        ReefscapeCoralOnFly outtakedNote = new ReefscapeCoralOnFly(
            // Obtain robot position from drive simulation
            CommandSwerveDrivetrain.getInstance().getMapleSimSwerveDrivetrain().mapleSimDrive
                .getSimulatedDriveTrainPose().getTranslation(),
            // The scoring mechanism is installed at (x,y) (meters) on the robot
            new Translation2d(carriagePose.getX() +
                EndEffectorConstants.DistanceFromCarriage.in(Meters) * Trig.cosizzle(endEffectorAngle), 0),
            // Obtain robot speed from drive simulation
            CommandSwerveDrivetrain.getInstance().getMapleSimSwerveDrivetrain().mapleSimDrive
                .getDriveTrainSimulatedChassisSpeedsFieldRelative(),
            // Obtain robot facing from drive simulation
            CommandSwerveDrivetrain.getInstance().getMapleSimSwerveDrivetrain().mapleSimDrive
                .getSimulatedDriveTrainPose().getRotation(),
            // The height at which the coral is ejected
            Meters.of(carriagePose.getZ() +
                EndEffectorConstants.DistanceFromCarriage.in(Meters) * Trig.sizzle(endEffectorAngle)),
            // The initial speed of the coral
            MetersPerSecond.of(EndEffectorConstants.OuttakeCoralSpeed.get()),
            // ejection angle
            armAngle.plus(EndEffectorConstants.AngleFromArmWrtGround));
        SimulatedArena.getInstance().addGamePieceProjectile(outtakedNote);
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.sensorValue = intakeSimulation.getGamePiecesAmount() != 0 ? (int) EndEffectorConstants.IRThreshold.get()
            : 0;

        inputs.motorConnected = true;
        inputs.motorStatorCurrent = motor.getCurrentDrawAmps();
        inputs.motorSupplyCurrent = 0.0d;
    }

    @Override
    public void setPower(double power) {
        motor.setInputVoltage(power * RobotController.getBatteryVoltage());
        if (power > 0) {
            intakeSimulation.startIntake();
        }
        if (power == 0) {
            intakeSimulation.stopIntake();
        }
        if (power < 0) {
            if (intakeSimulation.getGamePiecesAmount() > 0) {
                intakeSimulation.obtainGamePieceFromIntake();
                outtakeNote();
            }
        }
    }
}
