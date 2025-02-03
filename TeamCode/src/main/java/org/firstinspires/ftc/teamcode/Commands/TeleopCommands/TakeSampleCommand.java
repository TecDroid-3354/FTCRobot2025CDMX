package org.firstinspires.ftc.teamcode.Commands.TeleopCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.GripperAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Slider;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.SliderAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Wrist;

public class TakeSampleCommand extends CommandBase {
    private boolean finish = false;
    Slider slider;
    SliderAngle sliderAngle;
    Gripper gripper;
    GripperAngle gripperAngle;
    Arm arm;
    Wrist wrist;

    public TakeSampleCommand(Arm arm,
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
        //sliderAngle.goToPosIntakePosition();
        arm.goToPosition(Constants.Arm.sampleTakePosition);
        wrist.goToPosition(Constants.Wrist.intakePosition);
        slider.goToIntakePosition();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
        }
        gripper.close();
        try {
            Thread.sleep(650);
        } catch (InterruptedException e) {
        }
        arm.goToPosition(Constants.Arm.intakePosition);
        finish = true;

    }

    @Override
    public boolean isFinished() {
        return finish;
    }

}
