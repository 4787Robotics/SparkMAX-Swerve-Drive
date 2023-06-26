// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16448_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

public class TestSwerveModule extends SubsystemBase {
  private CANSparkMax turnMotor, moveMotor;
  private SparkMaxPIDController turnPIDController;
  private double setPoint, currentAngle, processVariable;

  private SparkMaxAbsoluteEncoder turnEncoder;

  private double currentModuleVelocity; //meters per second
  private double currentModuleAngle; //in radians

  private double WHEEL_X, WHEEL_Y;
  private double MODULE_NUMBER, TURN_MOTOR_ID, MOVE_MOTOR_ID;
  /** Creates a new ExampleSubsystem. */
  public TestSwerveModule(int MODULE_NUMBER, int TURN_MOTOR_ID, int MOVE_MOTOR_ID, double WHEEL_X, double WHEEL_Y) {
    this.WHEEL_X = WHEEL_X;
    this.WHEEL_Y = WHEEL_Y;

    this.MODULE_NUMBER = MODULE_NUMBER;
    this.TURN_MOTOR_ID = TURN_MOTOR_ID;
    this.MOVE_MOTOR_ID = MOVE_MOTOR_ID;

    turnMotor = new CANSparkMax(TURN_MOTOR_ID, MotorType.kBrushless);
    moveMotor = new CANSparkMax(MOVE_MOTOR_ID, MotorType.kBrushless);
    //change as needed
    turnMotor.setInverted(true);
    moveMotor.setInverted(true);

    turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    moveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    //sets max current
    turnMotor.setSmartCurrentLimit(20); //20 is the absolute max
    moveMotor.setSmartCurrentLimit(30); //30 is the absolute max
    //limits acceleration
    turnMotor.setOpenLoopRampRate(1);
    moveMotor.setOpenLoopRampRate(1);

    turnEncoder = turnMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    turnPIDController = turnMotor.getPIDController();
    turnPIDController.setFeedbackDevice(turnEncoder);

    turnEncoder.setPositionConversionFactor(1);
    turnEncoder.setVelocityConversionFactor(1/60);


    setPoint = 0;
    currentAngle = setPoint * 360;

    turnPIDController.setP(TURN_P);
    turnPIDController.setI(TURN_I);
    turnPIDController.setD(TURN_D);
    turnPIDController.setIZone(TURN_IZ);
    turnPIDController.setFF(TURN_FF);
    turnPIDController.setOutputRange(TURN_MIN_OUTPUT, TURN_MAX_OUTPUT);
    turnPIDController.setPositionPIDWrappingMinInput(TURN_MIN_OUTPUT);
    turnPIDController.setPositionPIDWrappingMaxInput(TURN_MAX_OUTPUT);
    turnPIDController.setPositionPIDWrappingEnabled(true);
  } 

  public double getWheelX() {
    return WHEEL_X;
  }

  public double getWheelY() {
    return WHEEL_Y;
  }

  public void setTurnPID(double setPoint) {
    turnPIDController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    processVariable = turnEncoder.getVelocity();
  }

  public void setTurnMotor(double speed) {
    turnMotor.set(speed);
  }
  
  public void setMoveMotor(double speed) {
    moveMotor.set(speed);
  }

  public CANSparkMax getTurnMotorReference() {
    return turnMotor;
  }

  public CANSparkMax getMoveMotorReference() {
    return moveMotor;
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

  public double getTurnCurrentVelocityRotationsPerSecond() {
    return turnEncoder.getVelocity();
  }

  public double getCurrentMoveVelocityRotationsPerSecond() {
    return moveMotor.getEncoder().getVelocity();
  }

  public double getCurrentMoveVelocityInchesPerSecond() {
    return getCurrentMoveVelocityRotationsPerSecond() * MOVE_WHEEL_CIRCUMFERENCE / MOVE_MOTOR_GEAR_RATIO;
  }

  public void updateShuffleBoard() {
    //SmartDashboard.putNumber("Turn Motor Velocity", turnMotor.getEncoder().getVelocity());
    //SmartDashboard.putNumber("Turn Motor Position", turnMotor.getEncoder().getPosition());
    //SmartDashboard.putNumber("Turn Motor Current", turnMotor.getOutputCurrent());
    //SmartDashboard.putNumber("Turn Motor Voltage", turnMotor.getBusVoltage());
    //SmartDashboard.putNumber("Turn Motor Temperature", turnMotor.getMotorTemperature());
    SmartDashboard.putNumber("Turn Motor " + this.MODULE_NUMBER + " Angle", currentAngle);
    SmartDashboard.putNumber("Turn Motor " + this.MODULE_NUMBER + " Setpoint", setPoint);
  }

  public void updatePIDValues() {
    if (SmartDashboard.getNumber("Turn Motor " + this.MODULE_NUMBER, 0) != setPoint) {
      setPoint = SmartDashboard.getNumber("Turn Motor " + this.MODULE_NUMBER, 0);
    }
  }

  public void turnToPoint(double newSetPoint) {
    //System.out.println("setpoint: " + newSetPoint);
    this.setPoint = newSetPoint;
    double currentPoint = getTurnEncoder();

    /*if ((newSetPoint - currentPoint) >= 0.5) {
      if (-1 /(2 * (1 / (newSetPoint - currentPoint))) > -0.5) {
        setTurnMotor(-1 /(2 * (1 / (newSetPoint - currentPoint))));
      }
    } else {
      if (1 /(2 * (1 / (newSetPoint - currentPoint))) < 0.5) {
        setTurnMotor(1 /(2 * (1 / (newSetPoint - currentPoint))));
      }
    }*/

    if (Math.max(newSetPoint, currentPoint) == currentPoint) {
      if ((currentPoint - newSetPoint) >= 0.5) {
          setTurnMotor(-1 /(4 * (1 / (currentPoint - newSetPoint))));
          //setTurnMotor(-0.1);
      } else {
          setTurnMotor(1 /(4 * (1 / (currentPoint - newSetPoint))));
          //setTurnMotor(0.1);
      }
    } else {
      if ((newSetPoint - currentPoint) >= 0.5) {
          setTurnMotor(-1 /(4 * (1 / (newSetPoint - currentPoint))));
          //setTurnMotor(-0.1);
      } else {
          setTurnMotor(1 /(4 * (1 / (newSetPoint - currentPoint))));
          //setTurnMotor(0.1);
      }
    }
  }

  @Override
  public void periodic() {
    //turnToPoint(0.5);
    //setMoveMotor(0.5);
    //setTurnMotor(0.1);
    //setPoint = 0.5; //set target is 0.75
    //turnPIDController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
    //setTurnMotor(0.5); //positive clockwise //0.5 max
    currentAngle = setPoint * 360;
    System.out.println("Module" + " setpoint = " + setPoint);
    //System.out.println("MODULE POWER: " + this.turnMotor.getOutputCurrent());
    //System.out.println("Module: " + MODULE_NUMBER);
    //System.out.println("PID output: " + turnEncoder.getVelocity());
  }
}
