package frc.subsystems.coralintake;

import static edu.wpi.first.units.Units.Kilograms;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.constants.Subsystems.CoralIntakeConstants;
import frc.utils.math.Algebra;

public class CoralIntakeIOSim implements CoralIntakeIO {
    private SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getNeo550(1),
        CoralIntakeConstants.RotorToArmRatio,
        SingleJointedArmSim.estimateMOI(Units.inchesToMeters(6), CoralIntakeConstants.Weight.in(Kilograms)),
        Units.inchesToMeters(6), CoralIntakeConstants.GroundAngle.radians(), CoralIntakeConstants.StowAngle.radians(),
        true, Math.PI / 2);
    private DCMotorSim intakeMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 1, 1),
        DCMotor.getNeo550(1));

    private double appliedVolts;
    private double appliedIntakeVolts;

    public CoralIntakeIOSim() {

    }

    @Override
    public void updateInputs(CoralIntakeIOInputsAutoLogged inputs) {
        if (pivotSim == null || intakeMotor == null) return;
        pivotSim.update(0.02);

        inputs.pivotConnected = true;
        inputs.appliedVoltage = appliedVolts;

        inputs.intakeConnected = true;
        inputs.appliedIntakeVolts = appliedIntakeVolts;

        inputs.throughborePosition = Algebra.linearMap(Units.radiansToDegrees(pivotSim.getAngleRads()),
            CoralIntakeConstants.GroundAngle.get(), CoralIntakeConstants.StowAngle.get(),
            CoralIntakeConstants.ThroughboreMin, CoralIntakeConstants.ThroughboreMax);
    }

    @Override
    public void setIntakeVoltage(double volts) {
        appliedIntakeVolts = volts;
        intakeMotor.setInputVoltage(volts);
    }

    @Override
    public void setPivotVoltage(double voltage) {
        if (RobotState.isDisabled()) {
            pivotSim.setInputVoltage(0);
            appliedVolts = 0;
            return;
        }
        appliedVolts = voltage;
        pivotSim.setInputVoltage(voltage);
    }
}
