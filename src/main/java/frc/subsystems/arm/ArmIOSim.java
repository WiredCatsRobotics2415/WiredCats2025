package frc.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.constants.Subsystems.ArmConstants;

public class ArmIOSim implements ArmIO {
    private double appliedVoltage;

    private final SingleJointedArmSim simArm = new SingleJointedArmSim(
        DCMotor.getFalcon500(2), ArmConstants.RotorToArmGearRatio,
        SingleJointedArmSim.estimateMOI(ArmConstants.ArmLengthMeters,
            ArmConstants.ApproximateMassKg),
        ArmConstants.ArmLengthMeters,
        Units.degreesToRadians(ArmConstants.potentiometerMinAngle),
        Units.degreesToRadians(ArmConstants.potentiometerMaxAngle), true,
        Units.degreesToRadians(ArmConstants.potentiometerMinAngle));

    public ArmIOSim() {
        simArm.setState(Units.degreesToRadians(ArmConstants.potentiometerMinAngle), 0);
    }

    @Override
    public void updateInputs(ArmIOInputsAutoLogged inputs) {
        simArm.update(0.02);

        inputs.leftConnected = true;
        inputs.rightConnected = true;
        inputs.appliedVoltage = appliedVoltage;
        inputs.position = Units.radiansToDegrees(simArm.getAngleRads());
    }

    @Override
    public void setVoltage(double voltage) {
        appliedVoltage = voltage;
        simArm.setInputVoltage(voltage);
    }

    // I refuse to implement the potentiometer modifying methods here.
    // In the sim world, the potentiometer is attached to the actual arm axle not geared to
    // it.
}
