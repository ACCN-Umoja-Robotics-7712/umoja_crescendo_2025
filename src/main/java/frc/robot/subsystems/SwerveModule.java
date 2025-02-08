package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


public class SwerveModule {
    public final SparkMax driveMotor, turnMotor;
    private final RelativeEncoder driveEncoder, turnEncoder;
    private final PIDController turnPIDController;
    public final CANcoder absoluteEncoder;
    public final int absoluteEncoderID;

    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean isAbsoluteEncoderReversed){   
        
        CANcoderConfiguration config = new CANcoderConfiguration();
        
        config.MagnetSensor.MagnetOffset = absoluteEncoderOffset;
        config.MagnetSensor.SensorDirection = isAbsoluteEncoderReversed ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        // config.MagnetSensor.AbsoluteSensorRangeValue = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        absoluteEncoder = new CANcoder(absoluteEncoderId, "rio");
        this.absoluteEncoderID = absoluteEncoderId;

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turnMotor = new SparkMax(turnMotorId, MotorType.kBrushless);

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.idleMode(IdleMode.kBrake);
        driveConfig.inverted(driveMotorReversed);
        driveConfig.smartCurrentLimit(30);
        
        SparkMaxConfig turnConfig = new SparkMaxConfig();
        turnConfig.idleMode(IdleMode.kBrake);
        turnConfig.inverted(turnMotorReversed);
        turnConfig.smartCurrentLimit(20);

        driveMotor.configure(driveConfig, null, null);
        turnMotor.configure(turnConfig, null, null);        
        
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        // driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        // driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        // turnEncoder.setPositionConversionFactor(ModuleConstants.kTurnEncoderRot2Rad);
        // turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurnEncoderRPM2RadPerSec);

        turnPIDController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
        
        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition()*ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningPosition() {
        return turnEncoder.getPosition()*ModuleConstants.kTurnEncoderRot2Rad;
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity()*ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }

    public double getTurningVelocity() {
        return turnEncoder.getVelocity()*ModuleConstants.kTurnEncoderRPM2RadPerSec;
    }

    public double getAbsoluteEncoderRad() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI;
    }

    public double getAbsoluteEncoderPosition() {
        return absoluteEncoder.getPosition().getValueAsDouble() * 2.0 * Math.PI;
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        resetTurn();
    }

    public void resetTurn(){
        double position = getAbsoluteEncoderRad();
        turnEncoder.setPosition(position);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // driveMotor.set(drivePIDcontroller.calculate())
        // TODO: CHANGE THIS TO PID

        //fill canbus
        // SmartDashboard.putNumber("ID (DRIVE) "+absoluteEncoderID + " TEMP: ", driveMotor.getMotorTemperature());
        // SmartDashboard.putNumber("ID (TURN) "+absoluteEncoderID + " TEMP: ", turnMotor.getMotorTemperature());

        // SmartDashboard.putNumber("ID: " + absoluteEncoderID, Math.toDegrees(getTurningPosition()));
        // SmartDashboard.putNumber("GOAL: " + absoluteEncoderID, Math.toDegrees(state.angle.getRadians()));
        // SmartDashboard.putNumber("Set motor percent: " + absoluteEncoderID, turnPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
        
        turnMotor.set(turnPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
        //turnMotor.set(turnPidController.calculate(getTurningPosition(), state.angle.getDegrees()));
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            Rotation2d.fromRadians(getTurningPosition()));
      }
}