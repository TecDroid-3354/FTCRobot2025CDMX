package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.Constants.Ids;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends SubsystemBase {
    private Telemetry telemetry;
    private final Servo servo;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        servo = hardwareMap.get(Servo.class, Ids.armServo);
        //goToPosition(0.0);
    }

    public void goToPosition(double position) {
        servo.setPosition(position / 180.0);
    }

    public double getPosition() {
        return servo.getPosition() * 180.0;
    }

    public boolean isAtPosition(double position) {
        return position + 0.01 >= servo.getPosition() && servo.getPosition() >= position - 0.01;
    }

    public void servoInfo(){
        telemetry.addData("Arm Info:", this.getPosition());
    }
}
