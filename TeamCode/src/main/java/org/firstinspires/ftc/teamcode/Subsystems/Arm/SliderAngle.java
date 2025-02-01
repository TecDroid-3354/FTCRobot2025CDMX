package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Constants.Constants.Ids;

public class SliderAngle extends SubsystemBase {
    private final DcMotorEx rightMotor;
    private final DcMotorEx leftMotor;
    private final Telemetry telemetry;
    private final PIDController positionPIDController;

    private double targetPosition = 1.0;
    private boolean goToPositionAllowed = false;
    private boolean homeState = false;
    private boolean intakeState = false;
    private boolean basketState = false;

    public SliderAngle(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        rightMotor = hardwareMap.get(DcMotorEx.class, Ids.sliderAngleRightMotorId);
        leftMotor = hardwareMap.get(DcMotorEx.class, Ids.sliderAngleLeftMotorId);

        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(4.0, 0.0, 0.0, 0.0));
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(4.0, 0.0, 0.0, 0.0));

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        positionPIDController = new PIDController(0.02, 0.0, 0.0);
    }

    public void setPower(double power) {
        rightMotor.setPower(power);
        leftMotor.setPower(power);
    }

    public double getAbsoluteEncoderPosition() {
        // Ticks to rotation to degrees
        return leftMotor.getCurrentPosition() * Constants.SliderAngle.ticksPerRev * 360.0;
    }

    public void goToPosition(double position) {
        double currentPosition = getAbsoluteEncoderPosition();
        double velocity = positionPIDController.calculate(currentPosition, position);
        velocity = Math.max(Math.min(1.0, velocity), -1.0);

        /* avoid the overheating of the motors
        boolean isInRestState = homeState || basketState;
        if (isInRestState &&
                (position + Constants.SliderAngle.positionTolerance >= currentPosition &&
                        currentPosition >= position - Constants.SliderAngle.positionTolerance)) {
            stopMotors();
        } else {
            setPower(velocity);
        }*/

        setPower(velocity);

    }

    public void setTargetPosition(double position){
        targetPosition = position;
    }

    public void goToHomePosition() {
        targetPosition = Constants.SliderAngle.homePosition;

        // change states
        homeState = true;
        intakeState = false;
        basketState = false;
    }

    public void goToIntakePosition() {
        targetPosition = Constants.SliderAngle.intakePosition;

        // change states
        homeState = false;
        intakeState = true;
        basketState = false;
    }
    public void goToPosIntakePosition() {
        targetPosition = Constants.SliderAngle.posIntakePosition;

        // change states
        homeState = false;
        intakeState = false;
        basketState = false;
    }

    public void goToQuesadillaPosition() {
        targetPosition = Constants.SliderAngle.quesadillaPosition;

        // change states
        homeState = false;
        intakeState = false;
        basketState = false;
    }

    public void goToSpecimenPosition() {
        targetPosition = Constants.SliderAngle.specimenScorePosition;

        // change states
        homeState = false;
        intakeState = false;
        basketState = false;
    }

    public void goToBasketPosition() {
        targetPosition = Constants.SliderAngle.basketScorePosition;

        // change states
        homeState = false;
        intakeState = false;
        basketState = true;
    }

    public boolean isAtPosition(double setPoint) {
        double position = getAbsoluteEncoderPosition();
        return setPoint + 0.8 >= position && position >= setPoint - 0.8;
        //return position >= setPoint - 0.8;
    }

    @Override
    public void periodic() {
        goToPosition(targetPosition);
    }

    public void stopMotors() {
        setPower(0.0);
    }

    public void encoderData() {
        telemetry.addData("Absolute Encoder", getAbsoluteEncoderPosition());
        telemetry.addData("Power", rightMotor.getPower());
    }

    public void motorData() {
        telemetry.addData("right", rightMotor.getCurrentPosition());
        telemetry.addData("left", getAbsoluteEncoderPosition());
    }
}
