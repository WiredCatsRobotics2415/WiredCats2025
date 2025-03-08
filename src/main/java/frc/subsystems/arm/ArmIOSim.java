package frc.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

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
        ArmConstants.MinDegreesFront.in(Radians), ArmConstants.MaxDegreesBack.in(Radians), true, Math.PI / 2);

    public ArmIOSim() {
        System.out.println("Arm MOI: " + moi);
        System.out.println("Max degrees (back): " + ArmConstants.MaxDegreesBack.in(Radians));
        System.out.println("min degrees (front): " + ArmConstants.MinDegreesFront.in(Radians));
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        simArm.update(0.02);

        inputs.motorConnected = true;
        inputs.motorSupplyCurrent = Amps.of(simArm.getCurrentDrawAmps());
        inputs.appliedVoltage = Volts.of(appliedVoltage);

        inputs.throughborePosition = Algebra.linearMap(Units.radiansToDegrees(simArm.getAngleRads()),
            ArmConstants.MinDegreesFront.in(Degrees), ArmConstants.MaxDegreesBack.in(Degrees),
            ArmConstants.ThroughboreMin, ArmConstants.ThroughboreMax);
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
