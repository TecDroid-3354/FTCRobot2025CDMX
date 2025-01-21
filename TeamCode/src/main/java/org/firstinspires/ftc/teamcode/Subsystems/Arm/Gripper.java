package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Constants.Constants.Ids;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Gripper extends SubsystemBase {
    private Boolean isOpen = false;
    public Telemetry telemetry;
    public final Servo servo;
    boolean buttonOpenCloseIsPressed = false;

    public Gripper(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        servo = hardwareMap.get(Servo.class, Ids.gripperServo);
        close();
    }

    public void goToPosition(double position) {
        servo.setPosition(position / 180.0);
    }

    public void open() {
        goToPosition(Constants.Gripper.openPositon);
        isOpen = true;
    }

    public void close() {
        goToPosition(Constants.Gripper.closePosition);
        isOpen = false;
    }

    public void openClose(boolean button) {
        if (button && !buttonOpenCloseIsPressed){
            buttonOpenCloseIsPressed = true;

            // Open and close the gripper acording to his state
            if (isOpen) {
                close();
            } else {
                open();
            }
        }

        if (!button) {
            buttonOpenCloseIsPressed = false;
        }

        isOpen = !isOpen;
    }

    public double getPosition() {
        return servo.getPosition() * 180.0;
    }

    public boolean isClosed() {
        return isAtPosition(Constants.Gripper.closePosition);
    }

    public boolean isAtPosition(double position) {
        return position + 0.01 >= servo.getPosition() && servo.getPosition() >= position - 0.01;
    }

    public void servoInfo(){
        telemetry.addData("Gripper Info", this.getPosition());
    }
}
