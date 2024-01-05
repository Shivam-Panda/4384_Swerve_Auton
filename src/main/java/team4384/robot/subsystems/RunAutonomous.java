package team4384.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RunAutonomous extends SequentialCommandGroup {
    Swerve s_Swerve;
    String pathName;

    public RunAutonomous(Swerve s_Swerve, String pathName) {
        this.s_Swerve = s_Swerve;
        this.pathName = pathName;
        addRequirements(s_Swerve);
        addCommands(
                new Autonomous(s_Swerve, pathName)
        );
    }
}
