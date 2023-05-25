package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import frc.robot.subsystems.TestSwerveModule;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestSwerve extends SubsystemBase {
    final int totalSwerveModules = 1;
    private TestSwerveModule[] swerveModules = new TestSwerveModule[totalSwerveModules];

    public TestSwerve() {
        for (int i = 0; i < totalSwerveModules; i++) {
            swerveModules[i] = new TestSwerveModule((i*2)+2, (i*2) + 1);
            swerveModules[i].resetMoveEncoder(); //set initial position to 0
        }
        //swerveModules[0] = new TestSwerveModule(TEST_TURN_MOTOR_ID, TEST_MOVE_MOTOR_ID);

        System.out.println("Swerve Initialized");
    }

    public void setSwerveModule(int swerveModule, double turnSpeed, double moveSpeed) {
        swerveModules[swerveModule].setTurnMotor(turnSpeed);
        swerveModules[swerveModule].setMoveMotor(moveSpeed);
    }

    public void moveAllSwerveModules(double turnSpeed, double moveSpeed) {
        System.out.println("Moving all swerve modules");
        for (int i = 0; i < totalSwerveModules; i++) {
            swerveModules[i].setTurnMotor(turnSpeed);
            swerveModules[i].setMoveMotor(moveSpeed);
        }
    }

    public void testMoveTurnPID(double setPoint) {
        swerveModules[0].setTurnPID(setPoint);
    }

    public void stopAllSwerveModules() {
        for (int i = 0; i < totalSwerveModules; i++) {
            swerveModules[i].setTurnMotor(0);
            swerveModules[i].setMoveMotor(0);
        }
    }

    public void updateDashboard() {
        //updates turn encoder values
        for (int i = 0; i < totalSwerveModules; i++) {
            SmartDashboard.putNumber("Swerve Module " + i + " Turn Encoder", swerveModules[i].getTurnEncoder());
        }

        //updates move encoder values
        for (int i = 0; i < totalSwerveModules; i++) {
            SmartDashboard.putNumber("Swerve Module " + i + " Move Encoder", swerveModules[i].getMoveEncoder());
        }
    }
} 

