package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Constants.Constants.Ids;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slider extends SubsystemBase {
    private final DcMotorEx rightMotor;
    private final DcMotorEx leftMotor;
    private final Telemetry telemetry;
    private final PIDController positionPIDController;
    private double targetPositon = 0.0;

    public Slider(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        rightMotor = hardwareMap.get(DcMotorEx.class, Ids.sliderRightMotorId);
        leftMotor = hardwareMap.get(DcMotorEx.class, Ids.sliderLeftMotorId);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        positionPIDController = new PIDController(0.45, 0.0, 0.3);
    }

    public double getRightEncoderPosition() {
        return rightMotor.getCurrentPosition() * Constants.Slider.encoderConvertionFactor *
                (Constants.Slider.rightEncoderReversed ? -1.0 : 1.0);
    }

    public double getLeftEncoderPosition() {
        return leftMotor.getCurrentPosition() * Constants.Slider.encoderConvertionFactor *
                (Constants.Slider.leftEncoderReversed ? -1.0 : 1.0);
    }
    public void setPower(double power) {
        rightMotor.setPower(power);
        leftMotor.setPower(power);
    }


    public void goToPosition(double position) {
        //double positionAvarage = (getRightEncoderPosition() + getLeftEncoderPosition()) / 2;
        double positionAvarage = getRightEncoderPosition();
        double velocity = -positionPIDController.calculate(positionAvarage, position);

        if (position + Constants.Slider.positionTolerance >= positionAvarage && positionAvarage >= position - Constants.Slider.positionTolerance){
            stopMotors();
        } else {
            setPower(velocity);
        }
    }

    public void goToHomePosition() {
        targetPositon = Constants.Slider.homePosition;
    }

    public void goToSpecimenPosition() {
        targetPositon = Constants.Slider.specimenScorePosition;
    }

    public void goToBasketPosition() {
        targetPositon = Constants.Slider.basketScorePosition;
    }

    @Override
    public void periodic() {
        goToPosition(targetPositon);
    }

    public void stopMotors() {
        setPower(0.0);
    }

    public void motorsData() {
        telemetry.addData("RightMotor", getRightEncoderPosition());
        telemetry.addData("LeftMotor", getLeftEncoderPosition());
    }
}
