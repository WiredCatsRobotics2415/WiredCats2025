package frc.subsystems.endeffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.constants.Subsystems.EndEffectorConstants;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import frc.subsystems.arm.Arm;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import frc.utils.Visualizer;
import edu.wpi.first.math.geometry.Pose3d;
import frc.utils.math.Trig;
import edu.wpi.first.units.measure.Angle;


public class EndEffectorIOSim implements EndEffectorIO {
    private DCMotorSim motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 1, 1),
        DCMotor.getNeo550(1));
    private final Arm armSubsystem = Arm.getInstance();

    private final IntakeSimulation intakeSimulation;
    private final ReefscapeCoralOnFly outtakeSimulation;

    public EndEffectorIOSim() {
        intakeSimulation = IntakeSimulation.OverTheBumperIntake("Coral",
            CommandSwerveDrivetrain.getInstance().getMapleSimSwerveDrivetrain().mapleSimDrive, Inches.of(9.5),
            Inches.of(11.3), IntakeSide.BACK, 1);

        Pose3d carriagePose = Visualizer.getActualCarriagePose();
        Angle armAngle = armSubsystem.getMeasurement();
        Angle endEffectorAngle = armAngle.plus(EndEffectorConstants.AngleFromArmWrtCarraige);

        outtakeSimulation = new ReefscapeCoralOnFly(
           // Obtain robot position from drive simulation
           CommandSwerveDrivetrain.getInstance().getMapleSimSwerveDrivetrain().mapleSimDrive
               .getSimulatedDriveTrainPose().getTranslation(),
           // The scoring mechanism is installed at (x,y) (meters) on the robot
           new Translation2d(carriagePose.getX() + EndEffectorConstants.DistanceFromCarriage.in(Meters) * Trig.cosizzle(endEffectorAngle), 0),
           // Obtain robot speed from drive simulation
           CommandSwerveDrivetrain.getInstance().getMapleSimSwerveDrivetrain().mapleSimDrive
               .getDriveTrainSimulatedChassisSpeedsFieldRelative(),
           // Obtain robot facing from drive simulation
           CommandSwerveDrivetrain.getInstance().getMapleSimSwerveDrivetrain().mapleSimDrive
               .getSimulatedDriveTrainPose().getRotation(),
           // The height at which the coral is ejected
           Meters.of(carriagePose.getZ() + EndEffectorConstants.DistanceFromCarriage.in(Meters) * Trig.sizzle(endEffectorAngle)),
           // The initial speed of the coral
           MetersPerSecond.of(EndEffectorConstants.OuttakeCoralSpeed.get()),
           // ejection angle
           armAngle.plus(EndEffectorConstants.AngleFromArmWrtGround)
        );
    }


    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.sensorValue = intakeSimulation.getGamePiecesAmount() != 0 ? (int) EndEffectorConstants.IRThreshold.get()
            : 0;

        inputs.motorConnected = true;
        inputs.motorStatorCurrent = Amps.of(motor.getCurrentDrawAmps());
        inputs.motorSupplyCurrent = Amps.of(0.0d);
        inputs.motorTemp = Celsius.of(0);
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
            intakeSimulation.obtainGamePieceFromIntake();
            SimulatedArena.getInstance().addGamePieceProjectile(outtakeSimulation);
        }
    }
}
