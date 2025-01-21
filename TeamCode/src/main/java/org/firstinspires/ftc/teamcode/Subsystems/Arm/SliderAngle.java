package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Constants.Constants.Ids;

public class SliderAngle extends SubsystemBase {
    private final DcMotorEx rightMotor;
    private final DcMotorEx leftMotor;
    private final Telemetry telemetry;
    private final PIDController positionPIDController;

    private double targetPosition = 1.0;

    public SliderAngle(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        rightMotor = hardwareMap.get(DcMotorEx.class, Ids.sliderAngleRightMotorId);
        leftMotor = hardwareMap.get(DcMotorEx.class, Ids.sliderAngleLeftMotorId);

        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        positionPIDController = new PIDController(0.025, 0.0, 0.0);
    }

    public void setPower(double power) {
        if ((getAbsoluteEncoderPosition() < Constants.SliderAngle.downLimit && power < 0 || getAbsoluteEncoderPosition() > Constants.SliderAngle.upLimit && power > 0)) {
            stopMotors();
        } else {
            rightMotor.setPower(power);
            leftMotor.setPower(power);
        }
    }

    public double getAbsoluteEncoderPosition() {
        // Ticks to rotation to degrees
        return leftMotor.getCurrentPosition() * Constants.SliderAngle.ticksPerRev * 360.0;
    }

    public void goToPosition(double position) {
        double currentPosition = getAbsoluteEncoderPosition();
        double velocity = positionPIDController.calculate(currentPosition, position);

        setPower(velocity);
    }

    public void goToHomePosition() {
        targetPosition = Constants.SliderAngle.homePosition;
    }

    public void goToIntakePosition() {
        targetPosition = Constants.SliderAngle.intakePosition;
    }

    public void goToSpecimenPosition() {
        targetPosition = Constants.SliderAngle.specimenScorePosition;
    }

    public void goToBasketPosition() {
        targetPosition = Constants.SliderAngle.basketScorePosition;
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
    }
}
