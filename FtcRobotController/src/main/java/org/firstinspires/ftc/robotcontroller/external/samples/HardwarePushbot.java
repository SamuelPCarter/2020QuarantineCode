package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  leftBackMotor:         "leftBackMotor"
 * Motor channel:  leftFrontMotor:        "leftFrontMotor"
 * Motor channel:  rightBackMotor:        "rightBackMotor"
 * Motor channel:  rightFrontMotor:       "rightFrontMotor"
 */
public class HardwarePushbot
{
    /* Public OpMode members. */
    public DcMotor  leftBackMotor   = null;
    public DcMotor  leftFrontMotor  = null;
    public DcMotor  rightBackMotor     = null;
    public DcMotor  rightFrontMotor     = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftBackMotor  = hwMap.get(DcMotor.class, "left_drive");
        leftFrontMotor = hwMap.get(DcMotor.class, "right_drive");
        rightBackMotor    = hwMap.get(DcMotor.class, "left_arm");
        rightFrontMotor    = hwMap.get(DcMotor.class, "left_arm");
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
 }

