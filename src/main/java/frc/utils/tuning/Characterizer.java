package frc.utils.tuning;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import lombok.Getter;

public class Characterizer {
    @Getter
    private static HashMap<String, Characterizer> characterizers = new HashMap<String, Characterizer>();

    public ArrayList<Command> commands = new ArrayList<Command>();

    public Characterizer(String subsystemName) {
        characterizers.put(subsystemName, this);
    }
}
