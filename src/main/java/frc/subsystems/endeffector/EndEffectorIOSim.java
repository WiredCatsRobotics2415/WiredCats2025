package frc.subsystems.endeffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.constants.Subsystems.EndEffectorConstants;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

public class EndEffectorIOSim implements EndEffectorIO {
    private DCMotorSim motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 1, 1),
        DCMotor.getNeo550(1));
    private DIOSim sensorInput = new DIOSim(new DigitalInput(EndEffectorConstants.IRSensorPort));

    private final IntakeSimulation intakeSimulation;

    public EndEffectorIOSim() {
        intakeSimulation = IntakeSimulation.OverTheBumperIntake("Coral",
            CommandSwerveDrivetrain.getInstance().getMapleSimSwerveDrivetrain().mapleSimDrive, Inches.of(9.5),
            Inches.of(11.3), IntakeSide.BACK, 1);
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.sensorValue = sensorInput.getValue() ? EndEffectorConstants.IRThreshold : 0;

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
        }
    }
}
