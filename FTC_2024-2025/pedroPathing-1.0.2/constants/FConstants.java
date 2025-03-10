package org.firstinspires.ftc.teamcode.NewPedroPathing.pedroPathing.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.THREE_WHEEL;

        FollowerConstants.leftFrontMotorName = "FL";
        FollowerConstants.leftRearMotorName = "BL";
        FollowerConstants.rightFrontMotorName = "FR";
        FollowerConstants.rightRearMotorName = "BR";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 5.4;

        FollowerConstants.xMovement = 56.0661;
        FollowerConstants.yMovement = 44.2474;

        FollowerConstants.forwardZeroPowerAcceleration = -55.23003;
        FollowerConstants.lateralZeroPowerAcceleration = -68.46416;

        FollowerConstants.translationalPIDFCoefficients = new CustomPIDFCoefficients(0.2,0,0.02,0);
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients = new CustomPIDFCoefficients(0.028,0,0.028,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients = new CustomPIDFCoefficients(1.2,0,0.064,0);
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients = new CustomPIDFCoefficients(2,0,0.063,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.0025,0,0.00008,0.6,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.004,0,0.00003,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0008;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;

        FollowerConstants.useBrakeModeInTeleOp = true;
    }
}
