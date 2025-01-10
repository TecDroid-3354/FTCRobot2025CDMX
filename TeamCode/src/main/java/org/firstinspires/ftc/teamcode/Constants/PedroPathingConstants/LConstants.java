package org.firstinspires.ftc.teamcode.Constants.PedroPathingConstants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.DriveEncoderConstants;
import com.pedropathing.localization.constants.ThreeWheelConstants;

// This acts as a method of updating ThreeWheelConstants without direct access to it.
public class LConstants { // This is how we change ThreeWheelConstants.
    static {
        DriveEncoderConstants.forwardTicksToInches = 1;
        DriveEncoderConstants.strafeTicksToInches = 1;
        DriveEncoderConstants.turnTicksToInches = 1;

        DriveEncoderConstants.robot_Width = 1;
        DriveEncoderConstants.robot_Length = 1;

        DriveEncoderConstants.leftFrontEncoderDirection = Encoder.FORWARD;
        DriveEncoderConstants.rightFrontEncoderDirection = Encoder.FORWARD;
        DriveEncoderConstants.leftRearEncoderDirection = Encoder.FORWARD;
        DriveEncoderConstants.rightRearEncoderDirection = Encoder.FORWARD;
    }
}

