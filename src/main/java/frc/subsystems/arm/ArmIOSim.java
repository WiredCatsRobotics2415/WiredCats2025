package frc.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.constants.Subsystems.ArmConstants;

public class ArmIOSim implements ArmIO {
    private double appliedVoltage;

    private final SingleJointedArmSim simArm = new SingleJointedArmSim(DCMotor.getFalcon500(1),
        ArmConstants.RotorToArmGearRatio,
        // Should the length be in meters?
        SingleJointedArmSim.estimateMOI(ArmConstants.EffectiveLengthInches / 39.37009424,
            ArmConstants.ApproximateMassKg),
        ArmConstants.EffectiveLengthInches / 39.37009424, Units.degreesToRadians(ArmConstants.MaxDegreesBack),
        Units.degreesToRadians(ArmConstants.MaxDegreesFront), true,
        Units.degreesToRadians(ArmConstants.MaxDegreesBack));

    public ArmIOSim() {
        simArm.setState(Units.degreesToRadians(ArmConstants.MaxDegreesBack), 0);
    }

    @Override
    public void updateInputs(ArmIOInputsAutoLogged inputs) {
        simArm.update(0.02);

        inputs.motorConnected = true;
        inputs.appliedVoltage = appliedVoltage;
        inputs.position = Units.radiansToDegrees(simArm.getAngleRads());
    }

    @Override
    public void setVoltage(double voltage) {
        appliedVoltage = voltage;
        simArm.setInputVoltage(voltage);
    }
}
