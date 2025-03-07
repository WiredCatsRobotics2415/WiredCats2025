package frc.subsystems.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.utils.tuning.TuneableNumber;
import lombok.Getter;

public class TuneableColor {
    @Getter private TuneableNumber red;
    @Getter private TuneableNumber green;
    @Getter private TuneableNumber blue;

    public TuneableColor(int red, int green, int blue, String name) {
        this.red = new TuneableNumber(red, "Colors/" + name + "/red");
        this.green = new TuneableNumber(green, "Colors/" + name + "/green");
        this.blue = new TuneableNumber(blue, "Colors/" + name + "/blue");
    }

    public TuneableColor(Color color, String name) {
        this((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), name);
    }

    public TuneableColor(Color8Bit color, String name) {
        this(color.red, color.green, color.blue, name);
    }

    public Color color() {
        return new Color(valid(red.get()), valid(blue.get()), valid(green.get()));
    }

    private int valid(double value) {
        return MathUtil.clamp((int) value, 0, 255);
    }
}
