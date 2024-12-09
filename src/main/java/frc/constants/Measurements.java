package frc.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.commands.Hotspot;
import java.util.ArrayList;

public class Measurements {
    private static final Translation3d BlueSpeaker = new Translation3d(0.225, 5.55, 2.1);
    private static final Translation3d RedSpeaker = new Translation3d(16.317, 5.55, 2.1);

    public static final Translation2d getSpeakerLocation() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red
                ? RedSpeaker.toTranslation2d()
                : BlueSpeaker.toTranslation2d();
        }
        return BlueSpeaker.toTranslation2d();
    }

    public static final ArrayList<Hotspot> Hotspots = new ArrayList<Hotspot>();

    static {
        Hotspots.add(new Hotspot(136.5, 200));
        Hotspots.add(new Hotspot(155, 236.5));
        Hotspots.add(new Hotspot(136.5, 243.5));
        Hotspots.add(new Hotspot(241.5, 295));
        Hotspots.add(new Hotspot(241.5, 238));
        Hotspots.add(new Hotspot(241.5, 181));
        Hotspots.add(new Hotspot(194.5, 112.638));
        Hotspots.add(new Hotspot(194.5, 324));
    }

    public static final ArrayList<Translation3d> DefaultFieldNotes = new ArrayList<Translation3d>();

    static {
        // Blue alliance side
        DefaultFieldNotes.add(
            new Translation3d(Units.inchesToMeters(114), Units.inchesToMeters(161.638), 0));
        DefaultFieldNotes.add(
            new Translation3d(Units.inchesToMeters(114), Units.inchesToMeters(218.638), 0));
        DefaultFieldNotes.add(
            new Translation3d(Units.inchesToMeters(114), Units.inchesToMeters(275.638), 0));
        // Centerline
        DefaultFieldNotes.add(new Translation3d(Units.inchesToMeters(325.6115),
            Units.inchesToMeters(29.638), 0));
        DefaultFieldNotes.add(new Translation3d(Units.inchesToMeters(325.6115),
            Units.inchesToMeters(95.638), 0));
        DefaultFieldNotes.add(new Translation3d(Units.inchesToMeters(325.6115),
            Units.inchesToMeters(161.638), 0));
        DefaultFieldNotes.add(new Translation3d(Units.inchesToMeters(325.6115),
            Units.inchesToMeters(227.638), 0));
        DefaultFieldNotes.add(new Translation3d(Units.inchesToMeters(325.6115),
            Units.inchesToMeters(293.638), 0));
    }

    public static final double NoteDiameter = Units.inchesToMeters(14);

    public static final Transform2d IntakeDownTransform = new Transform2d(0.5525, 0,
        new Rotation2d());

    public static final Transform3d BackLLTransform = new Transform3d(
        new Translation3d(Units.inchesToMeters(-12.815), 0, Units.inchesToMeters(10)),
        new Rotation3d(Units.degreesToRadians(-90), Units.degreesToRadians(30), 0));
}
