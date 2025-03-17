package frc.subsystems.arm;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.constants.Subsystems.ArmConstants;
import frc.utils.math.Algebra;

public class ArmIOSim implements ArmIO {
    private double appliedVoltage;

    private final double moi = SingleJointedArmSim.estimateMOI(ArmConstants.EffectiveLength.in(Meters),
        ArmConstants.ApproximateMassKg);

    private final SingleJointedArmSim simArm = new SingleJointedArmSim(DCMotor.getKrakenX60(1),
        ArmConstants.RotorToArmGearRatio, moi, ArmConstants.EffectiveLength.in(Meters),
        Units.degreesToRadians(ArmConstants.MinDegreesFront), Units.degreesToRadians(ArmConstants.MaxDegreesBack), true,
        Math.PI / 2);

    public ArmIOSim() {
        System.out.println("Arm MOI: " + moi);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        simArm.update(0.02);

        inputs.motorConnected = true;
        inputs.motorSupplyCurrent = simArm.getCurrentDrawAmps();
        inputs.appliedVoltage = appliedVoltage;

        inputs.throughborePosition = Algebra.linearMap(Units.radiansToDegrees(simArm.getAngleRads()),
            ArmConstants.MinDegreesFront, ArmConstants.MaxDegreesBack, ArmConstants.ThroughboreMin,
            ArmConstants.ThroughboreMax);
    }

    @Override
    public void setVoltage(double voltage) {
        if (RobotState.isDisabled()) {
            simArm.setInputVoltage(0);
            appliedVoltage = 0;
            return;
        }
        appliedVoltage = voltage;
        simArm.setInputVoltage(voltage);
    }
}
