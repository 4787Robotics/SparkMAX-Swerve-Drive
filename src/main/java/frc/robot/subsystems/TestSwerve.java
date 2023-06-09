package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import java.util.Collection;

import frc.robot.subsystems.TestSwerveModule;
import edu.wpi.first.math.Vector;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

import ai.djl.modality.cv.translator.wrapper.InputStreamImagePreProcessor;
import edu.wpi.first.wpilibj.SPI;

public class TestSwerve extends SubsystemBase {
    private AHRS gyro;

    final int totalSwerveModules = 1;
    private TestSwerveModule[] swerveModules = new TestSwerveModule[totalSwerveModules];

    private boolean fieldCentric = true;

    private double robotAngle = 0; //0 radians is upp
    private double robotAngularVelocity = 0; //radians/second
    private double robotMoveX = 0; //meters/second
    private double robotMoveY = 0; //meters/second

  public TestSwerve() {
        gyro = new AHRS(SPI.Port.kMXP);
        gyro.reset();
        /*for (int i = 0; i < totalSwerveModules; i++) {
            swerveModules[i] = new TestSwerveModule(i+1, TURN_MOTOR_IDS[i], MOVE_MOTOR_IDS[i], WHEEL_POSITIONS[i][0], WHEEL_POSITIONS[i][1]);
            swerveModules[i].resetMoveEncoder(); //set initial position to 0
        }*/ //to test all swerve modules

        swerveModules[0] = new TestSwerveModule(1, TEST_TURN_MOTOR_ID, TEST_MOVE_MOTOR_ID, 0, 0);

        System.out.println("Swerve Initialized");
        /*double[][] calculation = calculateVRotate(5, 1, 1, 0);
        System.out.println(calculation[1][0] + " " + calculation[1][1]);*/
        /*for (int i = 0; i < totalSwerveModules; i++) {
            double[] controllerInput = {1, 1};
            double[][] calculationVStrafe = calculateVStrafe(controllerInput, 1.16);
            System.out.println(calculationVStrafe[0][0] + " " + calculationVStrafe[0][1]);
            double[][] calculationVRotate = calculateVRotate(5, wheelPositions[i][0], wheelPositions[i][1], 1.16);
            System.out.println(calculationVRotate[0][0] + " " + calculationVRotate[0][1]);
            double[] calculationVSumRobot = calculateVSum(calculationVStrafe[0], calculationVRotate[0]);
            System.out.println(calculationVSumRobot[0] + " " + calculationVSumRobot[1]);

            double[] calculateVDriveWheel = {calculateVDriveWheel(calculationVSumRobot[0], calculationVSumRobot[1])};
            System.out.println("Module" + (i + 1) + " speed = "+ calculateVDriveWheel[0]);
            double[] calculateVRotateWheel = {caculateVRotateWheel(calculationVSumRobot[0], calculationVSumRobot[1])};
            System.out.println("Module" + (i + 1) + " angle = "+ calculateVRotateWheel[0]);
        }*/

        //System.out.println("input strength" + inputStrength(0.51345, 0.41235));
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

    public double[][] calculateVRotate(double rotate, double WHEEL_X, double WHEEL_Y, double currentRobotAngleRadians) {
        //calculate robot velocity vector
        double[] positionVector = {WHEEL_X, WHEEL_Y};
        double positionVectorMagnitude = Math.sqrt(Math.pow(positionVector[0], 2) + Math.pow(positionVector[1], 2));
        double positionVectorAngle = Math.atan2(positionVector[1], positionVector[0]);
        double velocityVectorMagnitude = rotate * positionVectorMagnitude;
        double velocityVectorAngle = positionVectorAngle + (Math.PI / 2);
        double velocityVectorX = velocityVectorMagnitude * Math.cos(velocityVectorAngle);
        double velocityVectorY = velocityVectorMagnitude * Math.sin(velocityVectorAngle);
        double velocityVectorRobot[] = {velocityVectorX, velocityVectorY};

        //calculate field velocity vector
        //currentRobotAngleRadians is negative or positive -> clockwise is negative, counterclockwise is positive
        velocityVectorAngle = positionVectorAngle + (Math.PI / 2) + currentRobotAngleRadians;
        velocityVectorX = velocityVectorMagnitude * Math.cos(velocityVectorAngle);
        velocityVectorY = velocityVectorMagnitude * Math.sin(velocityVectorAngle);
        double velocityVectorField[] = {velocityVectorX, velocityVectorY};

        double[][] calculation = {velocityVectorRobot, velocityVectorField};

        return calculation;
    }

    public double[][] calculateVStrafe(double[] strafeVelocity, double currentRobotAngleRadians) {
        //calculate robot velocity vector
        double velocityVectorX = (Math.cos(currentRobotAngleRadians) * strafeVelocity[0]) + (Math.sin(currentRobotAngleRadians) * strafeVelocity[1]);
        double velocityVectorY = (-Math.sin(currentRobotAngleRadians) * strafeVelocity[0]) + (Math.cos(currentRobotAngleRadians) * strafeVelocity[1]);
        double velocityVectorRobot[] = {velocityVectorX, velocityVectorY};

        //calculate field velocity vector
        double velocityVectorField[] = {strafeVelocity[0], strafeVelocity[1]};
        double[][] calculation = {velocityVectorRobot, velocityVectorField};
        return calculation;
    }

    public double[] calculateVSum(double[] velocityVector1, double[] velocityVector2) {
        double velocityVectorX = velocityVector1[0] + velocityVector2[0];
        double velocityVectorY = velocityVector1[1] + velocityVector2[1];
        double velocityVector[] = {velocityVectorX, velocityVectorY};
        return velocityVector;
    }

    public double calculateVDriveWheel(double velocityVectorX, double velocityVectorY) {
        return Math.sqrt((velocityVectorY * velocityVectorY) + (velocityVectorX * velocityVectorX));
    }

    public double caculateVRotateWheel(double velocityVectorX, double velocityVectorY) {
        return Math.atan2(-velocityVectorX, velocityVectorY);
    }

    static double largest(double[] arr) {
        int i;
         
        // Initialize maximum element
        double max = arr[0];
         
        // Traverse array elements from second and
        // compare every element with current max
        for (i = 1; i < arr.length; i++)
            if (arr[i] > max)
                max = arr[i];
         
        return max;
    }
     

    /**
     * strafeX is a left/right vector in meters/second
     * strafeY is a forward/backward vector in meters/second
     * rotate is angular velocity in radians/second
     * @param strafeX
     * @param strafeY
     * @param rotate
     */
    public void drive(double moveX, double moveY, double rotate) {
        //System.out.println("Driving");
        //System.out.println("moveX: " + moveX + " moveY: " + moveY + " rotate: " + rotate);
        double driveInputStrength = Math.max(Math.abs(moveX), Math.abs(moveY));
        double rotateInputStrength = Math.abs(rotate);
        double[] VDriveWheels = new double[4];
        double[] vAdjustedDriveWheels = new double[4];
        double[] VRotateWheels = new double[4];
        double maxStrength = 0; //which module has the highest velocity

        for (int i = 0; i < totalSwerveModules; i++) {
            double[] controllerInput = {moveX, moveY};
            double currentRobotAngleRadians = convertYawToRadians(gyro.getYaw());
            double[][] calculationVStrafe = calculateVStrafe(controllerInput, currentRobotAngleRadians);
            //System.out.println(calculationVStrafe[0][0] + " " + calculationVStrafe[0][1]);
            double[][] calculationVRotate = calculateVRotate(rotate, WHEEL_POSITIONS[i][0], WHEEL_POSITIONS[i][1], currentRobotAngleRadians);
            //System.out.println(calculationVRotate[0][0] + " " + calculationVRotate[0][1]);
            double[] VStrafe = {calculationVStrafe[0][0], calculationVStrafe[0][1]};
            double[] VRotate = {calculationVRotate[0][0], calculationVRotate[0][1]};
            double[] adjustedVStrafe = {calculationVStrafe[0][0] * driveInputStrength, calculationVStrafe[0][1] * driveInputStrength};
            double[] adjustedVRotate = {calculationVRotate[0][0] * rotateInputStrength, calculationVRotate[0][1] * rotateInputStrength};
            double[] calculationVSumRobot = calculateVSum(VStrafe, VRotate);
            double[] adjustedCalculationVSumRobot = calculateVSum(adjustedVStrafe, adjustedVRotate);
            //System.out.println(calculationVSumRobot[0] + " " + calculationVSumRobot[1]);
            double calculateVDriveWheel = calculateVDriveWheel(calculationVSumRobot[0], calculationVSumRobot[1]);
            double calculateAdjustedVDriveWheel = calculateVDriveWheel(adjustedCalculationVSumRobot[0], adjustedCalculationVSumRobot[1]);
            //System.out.println("Module" + (i + 1) + " speed = "+ calculateVDriveWheel);
            double calculateVRotateWheel = caculateVRotateWheel(calculationVSumRobot[0], calculationVSumRobot[1]);
            //System.out.println("Module" + (i + 1) + " angle = "+ calculateVRotateWheel);

            vAdjustedDriveWheels[i] = calculateAdjustedVDriveWheel;
            VDriveWheels[i] = calculateVDriveWheel;
            VRotateWheels[i] = calculateVRotateWheel;
        }

        maxStrength = largest(VDriveWheels);

        for (int i = 0; i < totalSwerveModules; i++) {
            //System.out.println(vAdjustedDriveWheels[i] / maxStrength);
            if (maxStrength != 0) {
                //swerveModules[i].setMoveMotor(vAdjustedDriveWheels[i] / maxStrength);
                //System.out.println("Module" + (i + 1) + " adjusted speed = "+ vAdjustedDriveWheels[i] / maxStrength);
            } else {
                //swerveModules[i].setMoveMotor(0);
                //System.out.println("Module" + (i + 1) + " adjusted speed = "+ 0);
            }

            double adjustedVRotateWheels = 0;
            if (-((VRotateWheels[i]) / Math.PI) >= 0) {
                adjustedVRotateWheels =  1 - ((((VRotateWheels[i]) /Math.PI)) / -2);
            } else {
                adjustedVRotateWheels = ((VRotateWheels[i]) /Math.PI) / 2;
            }
            swerveModules[i].turnToPoint(adjustedVRotateWheels);
            //System.out.println("Module" + (i + 1) + " setpoint = "+ -(VRotateWheels[i]));
            //System.out.println("Module" + (i + 1) + " adjusted setpoint = "+ -((VRotateWheels[i]) /Math.PI));
        }
    }

    public double convertYawToDegrees(double yaw) {
        if (yaw <= 0) {
            return yaw + 360;
        } else {
            return yaw;
        }
    }

    public double convertDegreesToRadians(double degrees) {
        return degrees * (Math.PI / 180);
    }

    public double convertYawToRadians(double yaw) {
        return convertDegreesToRadians(convertYawToDegrees(yaw));
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

