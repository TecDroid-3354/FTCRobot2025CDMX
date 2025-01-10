package org.firstinspires.ftc.teamcode.Constants.PedroPathingConstants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants.Constants;

// This acts as a method of updating FollowerConstants without direct access to it.
public class FConstants { // This is how we change Follower Constants.
    static {
        // Select our localizer
        FollowerConstants.localizers = Localizers.DRIVE_ENCODERS;

        FollowerConstants.leftFrontMotorName = Constants.Ids.frontLeftId;
        FollowerConstants.leftRearMotorName = Constants.Ids.backLeftId;
        FollowerConstants.rightFrontMotorName = Constants.Ids.frontRightId;
        FollowerConstants.rightRearMotorName = Constants.Ids.backRightId;

        FollowerConstants.leftFrontMotorDirection = DcMotorEx.Direction.FORWARD;
        FollowerConstants.leftRearMotorDirection = DcMotorEx.Direction.FORWARD;
        FollowerConstants.rightFrontMotorDirection = DcMotorEx.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorEx.Direction.REVERSE;

    }
}

