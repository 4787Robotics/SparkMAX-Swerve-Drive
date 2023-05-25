// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16448_IMU.CalibrationTime;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;


public class TestSwerveModule extends SubsystemBase {
  private CANSparkMax turnMotor, moveMotor;
  private SparkMaxPIDController turnPIDController;
  private double setPoint, provessVariable;

  private SparkMaxAbsoluteEncoder turnEncoder;
  /** Creates a new ExampleSubsystem. */
  public TestSwerveModule(int TURN_MOTOR_ID, int MOVE_MOTOR_ID) {
    turnMotor = new CANSparkMax(TURN_MOTOR_ID, MotorType.kBrushless);
    moveMotor = new CANSparkMax(MOVE_MOTOR_ID, MotorType.kBrushless);
    //change as needed
    turnMotor.setInverted(true);
    moveMotor.setInverted(true);

    turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    moveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    //sets max current
    turnMotor.setSmartCurrentLimit(15);
    moveMotor.setSmartCurrentLimit(25);
    //limits acceleration
    turnMotor.setOpenLoopRampRate(1);
    moveMotor.setOpenLoopRampRate(1);

    turnEncoder = turnMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    turnPIDController = turnMotor.getPIDController();

    turnPIDController.setP(TURN_P);
    turnPIDController.setI(TURN_I);
    turnPIDController.setD(TURN_D);
    turnPIDController.setIZone(TURN_IZ);
    turnPIDController.setFF(TURN_FF);
    turnPIDController.setOutputRange(TURN_MIN_OUTPUT, TURN_MAX_OUTPUT);
  } 

  public void setTurnPID(double setPoint) {
    turnPIDController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    provessVariable = turnEncoder.getVelocity();
  }

  public void setTurnMotor(double speed) {
    turnMotor.set(speed);
  }
  
  public void setMoveMotor(double speed) {
    moveMotor.set(speed);
  }

  public void resetMoveEncoder() {
    moveMotor.getEncoder().setPosition(0);
  }

  public double getTurnEncoder() {
    return turnEncoder.getPosition();
  }

  public double getMoveEncoder() {
    return moveMotor.getEncoder().getPosition();
  }
}
