package frc.robot.subsystems;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GameConstants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRot,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort, 
        DriveConstants.kFrontRightTurningMotorPort, 
        DriveConstants.kFrontRightDriveReversed, 
        DriveConstants.kFrontRightTurningEncoderReversed, 
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRot, 
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveReversed, 
        DriveConstants.kBackLeftTurningEncoderReversed, 
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRot, 
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort, 
        DriveConstants.kBackRightTurningMotorPort, 
        DriveConstants.kBackRightDriveReversed,
        DriveConstants.kBackRightTurningEncoderReversed, 
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRot, 
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    // private AprilTagFieldLayout fieldAprilTags= AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    // public final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
    //     DriveConstants.kDriveKinematics, new Rotation2d(),
    //     new SwerveModulePosition[] {
    //       frontLeft.getPosition(),
    //       frontRight.getPosition(),
    //       backLeft.getPosition(),
    //       backRight.getPosition()
    //     }, new Pose2d());
    
    public final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics, new Rotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        }, new Pose2d()
    );
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();
    StructArrayPublisher<SwerveModuleState> swerveStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    StructArrayPublisher<Pose2d> allPointsPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("AllPosesArray", Pose2d.struct).publish();
    

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
                resetDriveEncoders();
                // frontRight.driveMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
                // frontRight.turnMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
            }catch (Exception e) {
            }
        }).start();
    
        RobotConfig config;
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
            e.printStackTrace();
            System.out.println("ROBOT GAVE UP PLEASE FIX CONFIG");
            return;
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> setModuleStatesFromSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(1, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(Constants.ModuleConstants.kPTurning, 0.0, 0.0) // Rotation PID constants
                ),
                config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public void resetDriveEncoders(){
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    // public void resetTurn(){
    //     frontLeft.resetTurn();
    //     frontRight.resetTurn();
    //     backLeft.resetTurn();
    //     backRight.resetTurn();
    // }

    public void zeroHeading(){
        gyro.reset();
    }

    public double getHeading(){
        //Changes the -180->0 to 180->360 (0 to 180 stays the same)
        double heading = Math.IEEEremainder(-gyro.getYaw(), 360);
        // double heading = (-gyro.getYaw() + 360)%180;
        if(heading < 0){
            heading = 180 + (180+heading);
        }
        SmartDashboard.putNumber("HEADING", heading);
        return heading;
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        System.out.println("ODOMETRY RESET");
        poseEstimator.resetPosition(Rotation2d.fromDegrees(gyro.getYaw()), new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        }, pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("FL", Math.toDegrees(frontLeft.getAbsoluteEncoderRad()));
        SmartDashboard.putNumber("FR", Math.toDegrees(frontRight.getAbsoluteEncoderRad()));
        SmartDashboard.putNumber("BL", Math.toDegrees(backLeft.getAbsoluteEncoderRad()));
        SmartDashboard.putNumber("BR", Math.toDegrees(backRight.getAbsoluteEncoderRad()));

        SmartDashboard.putNumber("T FL", Math.toDegrees(frontLeft.getTurningPosition()%360));
        SmartDashboard.putNumber("T FR", Math.toDegrees(frontRight.getTurningPosition()%360));
        SmartDashboard.putNumber("T BL", Math.toDegrees(backLeft.getTurningPosition()%360));
        SmartDashboard.putNumber("T BR", Math.toDegrees(backRight.getTurningPosition()%360));
        SmartDashboard.putNumber("YAW", gyro.getYaw());

        if(RobotContainer.gameState!=GameConstants.TeleOp){
            poseEstimator.update(Rotation2d.fromDegrees(-gyro.getYaw()),
            // odometer.update(getRotation2d(),
                new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            });

            // var result = RobotContainer.camera.getLatestResult();
            // boolean hasTargets = result.hasTargets();
            // SmartDashboard.putBoolean("Has Targets", hasTargets);
            // if(hasTargets){
            //     var imageCaptureTime = result.getTimestampSeconds();
            //     var camToTargetTrans = result.getBestTarget().getBestCameraToTarget();
            //     int id = result.getBestTarget().getFiducialId();
            //     var tagFromId = fieldAprilTags.getTagPose(id);
            //     if (tagFromId != null) {
            //         Pose3d tag = tagFromId.get();
            //         Pose3d poseTarget = new Pose3d(tag.getTranslation(), tag.getRotation());
            //         SmartDashboard.putNumber("X", tag.getX());
            //         SmartDashboard.putNumber("Y", tag.getY());
            //         SmartDashboard.putNumber("Z", tag.getZ());
            //         var camPose = poseTarget.transformBy(camToTargetTrans.inverse());
            //         poseEstimator.addVisionMeasurement(camPose.transformBy(PoseEstimatorConstants.kCameraToRobot).toPose2d(), imageCaptureTime);
            //     }
            // }

        String tagLimelightName = Constants.LimelightConstants.tagName;
        if (LimelightHelpers.getTargetCount(tagLimelightName) != 0 && RobotContainer.gameState == GameConstants.Robot) {
            Pose3d targetPose3d = LimelightHelpers.getTargetPose3d_RobotSpace(tagLimelightName);
            Double targetYaw = targetPose3d.getRotation().getMeasureAngle().baseUnitMagnitude();
            Double targetX = targetPose3d.getX();
    
            SmartDashboard.putNumber("target yaw", targetYaw);
            SmartDashboard.putNumber("target x", targetX);

            // lined up angle perfectly (turn off limelight)
            // TODO: Move this number to constants file
            Boolean alignedYaw = targetYaw <= 0.25;
            Boolean alignedX = Math.abs(targetX) <= 0.02;
            if (alignedX && alignedYaw){
                // TODO: Replace with LEDs ready for game
                LimelightHelpers.setLEDMode_ForceOff(Constants.LimelightConstants.gamePieceName);
            } else {
                // TODO: Replace with LEDs not game ready
                LimelightHelpers.setLEDMode_ForceOn(Constants.LimelightConstants.gamePieceName);
            }
        } else {
            // TODO: Replace with LEDs not game ready
            LimelightHelpers.setLEDMode_ForceOn(Constants.LimelightConstants.gamePieceName);
        }
        
        LimelightHelpers.SetRobotOrientation(tagLimelightName, poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(tagLimelightName);

        boolean doRejectUpdate = false;

        if (Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            doRejectUpdate = true;
        }
        if (mt2 != null) {
            if (mt2.tagCount == 0)
            {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate)
            {
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0,0,9999999));
                poseEstimator.addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds);
            }
        }
    
        posePublisher.set(poseEstimator.getEstimatedPosition());
        }
        ArrayList<Pose2d> allPoints = new ArrayList<>();
        for (Pose2d point: Constants.bluePickUpPositions) {
            allPoints.add(point);
            allPoints.add(offsetPoint(point, 3*Constants.Measurements.coralStationDivotOffset));
            allPoints.add(offsetPoint(point, -3*Constants.Measurements.coralStationDivotOffset));
        }
        for (Pose2d point: Constants.blueReefPositions) {
            allPoints.add(point);
            allPoints.add(offsetPoint(point, Constants.Measurements.branchOffset));
            allPoints.add(offsetPoint(point, -Constants.Measurements.branchOffset));
        }
        allPoints.add(Constants.blueProcessorPosition);
        
        for (Pose2d point: Constants.redPickUpPositions) {
            allPoints.add(point);
            allPoints.add(offsetPoint(point, 3*Constants.Measurements.coralStationDivotOffset));
            allPoints.add(offsetPoint(point, -3*Constants.Measurements.coralStationDivotOffset));
        }
        for (Pose2d point: Constants.redReefPositions) {
            allPoints.add(point);
            allPoints.add(offsetPoint(point, Constants.Measurements.branchOffset));
            allPoints.add(offsetPoint(point, -Constants.Measurements.branchOffset));
        }
        allPoints.add(Constants.redProcessorPosition);
        allPointsPublisher.set(allPoints.toArray(new Pose2d[0]));
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontRight.setDesiredState(desiredStates[0]);
        frontLeft.setDesiredState(desiredStates[1]);
        backRight.setDesiredState(desiredStates[2]);
        backLeft.setDesiredState(desiredStates[3]);
    }

    public void setModuleStatesFromSpeeds(ChassisSpeeds speeds){
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(moduleStates);
    }

    public Pose2d nearestPoint(boolean isProcessor, boolean hasCoral) {
        
         // default from center reef starting line
        Pose2d nearestPoint = Constants.RobotPositions.blueReefBackCenter21;

        boolean isBlue = true;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                isBlue = false;
                nearestPoint = Constants.RobotPositions.redReefBackCenter10;
            }
        }

        if (isProcessor) {
            if (isBlue) {
                return Constants.blueProcessorPosition;
            } else {
                return Constants.redProcessorPosition;
            }
        }

        double nearestDistanceSoFar = Double.POSITIVE_INFINITY;

        if (hasCoral) {
            Pose2d[] pointsToCheck;
            if (isBlue) {
                pointsToCheck = Constants.blueReefPositions;
            } else {
                pointsToCheck = Constants.redReefPositions;
            }
            
            for (Pose2d reefPoint: pointsToCheck) {
                double currDistance = poseEstimator.getEstimatedPosition().getTranslation().getDistance(reefPoint.getTranslation());
                if (currDistance <= nearestDistanceSoFar) {
                    nearestDistanceSoFar = currDistance;
                    nearestPoint = reefPoint;
                }
            }
        } else {
            Pose2d[] pointsToCheck;
            if (isBlue) {
                pointsToCheck = Constants.bluePickUpPositions;
            } else {
                pointsToCheck = Constants.redPickUpPositions;
            }
            
            for (Pose2d pickUpPoints: pointsToCheck) {
                double currDistance = poseEstimator.getEstimatedPosition().getTranslation().getDistance(pickUpPoints.getTranslation());
                if (currDistance <= nearestDistanceSoFar) {
                    nearestDistanceSoFar = currDistance;
                    nearestPoint = pickUpPoints;
                }
            }
        }
        
        return nearestPoint;
    }

    public Pose2d offsetPoint(Pose2d pose, double offset) {
        Transform2d transform = new Transform2d(0, offset, new Rotation2d(0));
        return pose.transformBy(transform);
    }
}
