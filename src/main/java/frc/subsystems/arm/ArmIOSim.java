package frc.subsystems.arm;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.constants.Subsystems.ArmConstants;

public class ArmIOSim implements ArmIO {
    private double appliedVoltage;

    private final SingleJointedArmSim simArm = new SingleJointedArmSim(DCMotor.getFalcon500(1),
        ArmConstants.RotorToArmGearRatio,
        // Should the length be in meters?
        SingleJointedArmSim.estimateMOI(ArmConstants.EffectiveLengthInches.div(39.37009424).in(Inches),
            ArmConstants.ApproximateMassKg),
        ArmConstants.EffectiveLengthInches.div(39.37009424).in(Inches), ArmConstants.MaxDegreesBack.in(Radians),
        ArmConstants.MaxDegreesFront.in(Radians), true, ArmConstants.MaxDegreesBack.in(Radians));

    public ArmIOSim() {
        simArm.setState(ArmConstants.MaxDegreesBack.in(Radians), 0);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        simArm.update(0.02);

        inputs.motorConnected = true;
        inputs.appliedVoltage = Volts.of(appliedVoltage);
        inputs.throughborePosition = Units.radiansToRotations(simArm.getAngleRads());
    }

    @Override
    public void setVoltage(double voltage) {
        appliedVoltage = voltage;
        simArm.setInputVoltage(voltage);
    }
}
