package org.firstinspires.ftc.teamcode.Commands.TeleopCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Sensors.ColorDetector;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.GripperAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Slider;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.SliderAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Wrist;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;

public class BasketScoreSecuence extends CommandBase {
    private boolean isAtPosition = false;
    Slider slider;
    SliderAngle sliderAngle;
    Gripper gripper;
    GripperAngle gripperAngle;
    Arm arm;
    Wrist wrist;

    public BasketScoreSecuence(Arm arm,
                               Slider slider,
                               SliderAngle sliderAngle,
                               Gripper gripper,
                               GripperAngle gripperAngle,
                               Wrist wrist) {

        this.arm = arm;
        this.slider = slider;
        this.gripper = gripper;
        this.gripperAngle = gripperAngle;
        this.sliderAngle = sliderAngle;
        this.wrist = wrist;

    }

    @Override
    public void execute() {
        // Score piece
        if (sliderAngle.isAtPosition(Constants.SliderAngle.homePosition) &&
                slider.isAtPosition(Constants.Slider.basketScorePosition)) {
            if (sliderAngle.isAtPosition(Constants.SliderAngle.basketScorePosition)) {
                gripper.open();
                try {Thread.sleep(1000);} catch (InterruptedException e) {}
                sliderAngle.goToHomePosition();
                slider.goToHomePosition();
                arm.goToPosition(Constants.Arm.homePositon);
                wrist.goToPosition(Constants.Wrist.homePositon);
                isAtPosition = true;
            } else {
                sliderAngle.goToBasketPosition();
                try {Thread.sleep(1000);} catch (InterruptedException e) {}
            }
        } else {
            // Go to basket position
            sliderAngle.goToHomePosition();
            if (sliderAngle.isAtPosition(Constants.SliderAngle.homePosition)) {
                slider.goToBasketPosition();
            } else {
                arm.goToPosition(Constants.Arm.basketScorePosition);
                wrist.goToPosition(Constants.Wrist.basketScorePosition);
            }
        }

    }

    @Override
    public boolean isFinished() {
        return isAtPosition;
    }

}
