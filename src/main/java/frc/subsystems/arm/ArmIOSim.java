package frc.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.constants.Subsystems.ArmConstants;

public class ArmIOSim implements ArmIO {
    private double appliedVoltage;

    private final double moi = SingleJointedArmSim.estimateMOI(ArmConstants.EffectiveLength.in(Meters),
        ArmConstants.ApproximateMassKg);

    private final SingleJointedArmSim simArm = new SingleJointedArmSim(DCMotor.getKrakenX60(1),
        ArmConstants.RotorToArmGearRatio, moi, ArmConstants.EffectiveLength.in(Meters),
        ArmConstants.MaxDegreesBack.in(Radians), ArmConstants.MaxDegreesFront.in(Radians), true, Math.PI / 2);

    public ArmIOSim() {
        System.out.println("Arm MOI: " + moi);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        simArm.update(0.02);

        inputs.motorConnected = true;
        inputs.motorSupplyCurrent = Amps.of(simArm.getCurrentDrawAmps());
        inputs.appliedVoltage = Volts.of(appliedVoltage);

        inputs.throughborePosition = Units.radiansToRotations(simArm.getAngleRads());
    }

    @Override
    public void setVoltage(double voltage) {
        appliedVoltage = voltage;
        simArm.setInputVoltage(voltage);
    }
}
