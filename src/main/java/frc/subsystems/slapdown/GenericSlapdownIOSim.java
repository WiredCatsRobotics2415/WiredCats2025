package frc.subsystems.slapdown;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

public class GenericSlapdownIOSim implements GenericSlapdownIO {
    private SingleJointedArmSim pivotSim;
    private DCMotorSim intakeMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 1, 1),
        DCMotor.getNeo550(1));
    private IntakeSimulation intakeSim;

    private boolean down;

    public GenericSlapdownIOSim() {}

    @Override
    public void updateInputs(GenericSlapdownIOInputsAutoLogged inputs) {
        if (pivotSim == null || intakeMotor == null) return;

        inputs.pivotConnected = true;

        inputs.intakeConnected = true;

        inputs.throughborePosition = Units.radiansToRotations(pivotSim.getAngleRads());
        inputs.sensorValue = intakeSim.getGamePiecesAmount() == 1 ? 0 : 100;

        down = pivotSim.getAngleRads() < Units.degreesToRadians(5);
    }

    public void configureSim(String targetGamePiece, Distance width, Distance lengthExtended, IntakeSide side,
        double rotorToGearRatio, Distance effectiveLength, Angle max, Angle min, Mass mass) {
        if (!targetGamePiece.equals("")) {
            intakeSim = IntakeSimulation.OverTheBumperIntake(targetGamePiece,
                CommandSwerveDrivetrain.getInstance().getMapleSimSwerveDrivetrain().mapleSimDrive, width,
                lengthExtended, side, 1);
        }

        pivotSim = new SingleJointedArmSim(DCMotor.getNeo550(1), rotorToGearRatio,
            SingleJointedArmSim.estimateMOI(effectiveLength.in(Meters), mass.in(Kilograms)), effectiveLength.in(Meters),
            min.in(Radians), max.in(Radians), true, Math.PI / 2);
    }

    @Override
    public void setIntakePower(double power) {
        intakeMotor.setInputVoltage(power * RoboRioSim.getVInVoltage());
        if (intakeSim == null) return;
        if (power > 0 && down) {
            intakeSim.startIntake();
        }
        if (power == 0) intakeSim.stopIntake();
        if (power < 0) {
            intakeSim.obtainGamePieceFromIntake();
        }
    }

    @Override
    public void setPivotVoltage(double voltage) {
        if (pivotSim == null) return;
        pivotSim.setInputVoltage(voltage);
    }
}
