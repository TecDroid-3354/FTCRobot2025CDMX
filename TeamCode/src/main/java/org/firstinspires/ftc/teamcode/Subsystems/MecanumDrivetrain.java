package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.Constants.Ids;
import org.firstinspires.ftc.teamcode.Constants.Constants.Velocities;
import org.opencv.core.Point;


public class MecanumDrivetrain extends SubsystemBase {
    // Telemetry
    Telemetry telemetry;

    // Motors
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    // Kinematics
    // Locations of the wheels relative to the robot center.
    Translation2d m_frontLeftLocation =
            new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation =
            new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation =
            new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation =
            new Translation2d(-0.381, -0.381);

    // Creating my kinematics object using the wheel locations.
    MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics
            (
                    m_frontLeftLocation, m_frontRightLocation,
                    m_backLeftLocation, m_backRightLocation
            );

    // PController
    PController alignPController;

    public MecanumDrivetrain (HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Motors
        frontLeftMotor = hardwareMap.dcMotor.get(Ids.frontLeftId);
        backLeftMotor = hardwareMap.dcMotor.get(Ids.backLeftId);
        frontRightMotor = hardwareMap.dcMotor.get(Ids.frontRightId);
        backRightMotor = hardwareMap.dcMotor.get(Ids.backRightId);

        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        alignPController = new PController(0.05);
    }

    public void drive(ChassisSpeeds speeds) {
        // Convert to wheel speeds
        MecanumDriveWheelSpeeds wheelSpeeds =
                m_kinematics.toWheelSpeeds(speeds);

        // Get the individual wheel speeds
        double frontLeft = wheelSpeeds.frontLeftMetersPerSecond;
        double frontRight = wheelSpeeds.frontRightMetersPerSecond;
        double backLeft = wheelSpeeds.rearLeftMetersPerSecond;
        double backRight = wheelSpeeds.rearRightMetersPerSecond;

        frontLeftMotor.setPower(frontLeft);
        frontRightMotor.setPower(frontRight);
        backLeftMotor.setPower(backLeft);
        backRightMotor.setPower(backRight);
    }

    public void basicDrive(double x, double y, double rx) {
        double yVelocity = y * Velocities.driveMaxVelocity;
        double xVelocity = x * Velocities.driveMaxVelocity;
        double rxVelocity = rx * Velocities.angularMaxVelocity;

        ChassisSpeeds speeds = new ChassisSpeeds(yVelocity, xVelocity, rxVelocity);
        drive(speeds);
    }

    public void driveFieldOriented(double x, double y, double rx, double robotHeading) {
        double yVelocity = y * Velocities.driveMaxVelocity;
        double xVelocity = x * Velocities.driveMaxVelocity;
        double rxVelocity = rx * Velocities.angularMaxVelocity;

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                yVelocity, xVelocity, rxVelocity, Rotation2d.fromDegrees(robotHeading)
        );
        drive(speeds);
    }

    public void  alignToGamePiece(Point closestDetection) {
        // Camera Center point
        Point centerOfCamera = new Point(160, 120);

        double xVelocity = alignPController.calculate(closestDetection.x, centerOfCamera.x);

        ChassisSpeeds speeds = new ChassisSpeeds(0.0, xVelocity, 0.0);
        drive(speeds);
    }

    public void motorsData() {
        telemetry.addData("FL", frontLeftMotor.getCurrentPosition());
        telemetry.addData("FR", frontRightMotor.getCurrentPosition());
        telemetry.addData("BL", backLeftMotor.getCurrentPosition());
        telemetry.addData("BR", backRightMotor.getCurrentPosition());
    }

    public void stopMotors() {
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
    }
}
