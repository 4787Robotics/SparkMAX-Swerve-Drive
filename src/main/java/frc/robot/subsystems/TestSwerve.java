package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import frc.robot.subsystems.TestSwerveModule;

public class TestSwerve extends SubsystemBase {
    final int totalSwerveModules = 4;
    private TestSwerveModule[] swerveModules = new TestSwerveModule[totalSwerveModules];

    public TestSwerve() {
        for (int i = 0; i < totalSwerveModules; i++) {
            swerveModules[i] = new TestSwerveModule(i*2, (i*2) + 1);
        }

        PrintCommand printCommand = new PrintCommand(swerveModules.toString());
        printCommand.schedule();
    }
}

