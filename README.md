# WiredCats2024 as an AdvantageKit project

[AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit) is a logging system for WPILib projects. Mechanical Advantage (team 6328) maintains it along with their log viewer [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope). They have the best robot code framework that I have seen out there, so I've replicated it using our competition code for the 2024 season in this repo.

Significant changes made to the 2024 code:
 - There is now a constants folder with better names for all classes/variables
 - OI keybind strings are now enums
 - Extensive simulation support for arm and simulated note shots
 - All subsystems are separated into logic and IO layers per Mechanical Advantage's recommendation
 - util is reorganized and any logic that should have been its own class is moved here

Future plans:
 - rework all commands in the commands/ folder so they look nice
 - add multiple limelight pose estimation support to the vision subsystem and make the pose estimation logic in SwerveDrive realistic
    - this means that the robot considers its acceleration when filtering pose estimates like ORBIT does
 - add the robot CAD files so others can use them in Ascope
 - write swerve drive from the ground up
    - the swerve drive class itself should still be compatible with the TunerConstants data classes, so that a generated TunerConstants file can be dropped in to work (and commenting out the swervedrivetrain constructor call at the bottom obviously)
    - should also have its own odometry stack like Mechanical Advantage but instead of multiplying current wheel speed by 0.02 seconds every loop to get wheel position change, just use the current wheel position
    - also the current CTRE swerve simulator does not work if the current limit is set to 40A so this bug should be avoided
    - also should modules to have different steer tunes (not drive tunes since it's not possible to tune them independently)
 - Add sysid commands to each class and put them on elastic (elastic works super well btw) when robot is in test mode
    - ideally test mode is its own class called "TuningMode" that operates at the same level of abstraction as RobotContainer does
      - CONSIDER using a global constant for this - more stable/clean than trying to detect a robot mode
      - Tuning mode should also prevent real IOs from calling optimizeBusUtilization()
    - also make all appropriate constants loggedtunablenumbers (like Mechanical Advantage does) and make them available for editing on elastic in test mode
      - see how mech adv made theirs for inspiration
 - network alerts
