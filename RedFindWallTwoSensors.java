/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;


/**
 * This file illustrates the concept of driving a path based on time.
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,

 */

@Autonomous(name = "RedFindWallTwoSensors", group = "Example")
//@Disabled
class RedFindWallTwoSensors extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareDefinition         robot   = new HardwareDefinition();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    ModernRoboticsI2cRangeSensor rangeSensor;


    private static double     FORWARD_SPEED = 0.3;
    private static final double      STRAFE_SPEED = 1;
    ColorSensor lineSensor;    // Hardware Device Object
    ColorSensor redBlueSensor;    // Hardware Device Object

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float lhsvValues[] = {0F,0F,0F};
    float rbhsvValues[] = {0F,0F,0F};

    @Override
    public void runOpMode() throws InterruptedException {

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
        // get a reference to our ColorSensor objects and set each sensor's address
        lineSensor = hardwareMap.colorSensor.get("line sensor");
        lineSensor.setI2cAddress(I2cAddr.create8bit(0x3a) );
        redBlueSensor = hardwareMap.colorSensor.get("color sensor");
        redBlueSensor.setI2cAddress(I2cAddr.create8bit(0x3c) );

        boolean bLedOn = true;

        // Set the LED in the beginning
        lineSensor.enableLed(bLedOn);
        redBlueSensor.enableLed(!bLedOn);

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to Start");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  strafe right for one point seven seconds
        robot.leftFrontWheel.setPower(-STRAFE_SPEED);
        robot.leftBackWheel.setPower(STRAFE_SPEED);
        robot.rightFrontWheel.setPower(STRAFE_SPEED);
        robot.rightBackWheel.setPower(-STRAFE_SPEED);
        runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.7)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }
        // This stops the the robot from moving
        robot.leftFrontWheel.setPower(0);
        robot.leftBackWheel.setPower(0);
        robot.rightFrontWheel.setPower(0);
        robot.rightBackWheel.setPower(0);
        // This makes the robot go backwards for .3 of a second
        robot.leftFrontWheel.setPower(-STRAFE_SPEED);
        robot.leftBackWheel.setPower(-STRAFE_SPEED);
        robot.rightFrontWheel.setPower(-STRAFE_SPEED);
        robot.rightBackWheel.setPower(-STRAFE_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }  // This makes the robot go backwards slowly for a fifth of a second
        robot.leftFrontWheel.setPower(-0.5*STRAFE_SPEED);
        robot.leftBackWheel.setPower(-0.5*STRAFE_SPEED);
        robot.rightFrontWheel.setPower(-0.5*STRAFE_SPEED);
        robot.rightBackWheel.setPower(-0.5*STRAFE_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.2)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }
        // This stops the the robot from moving for a second
        robot.leftFrontWheel.setPower(0);
        robot.leftBackWheel.setPower(0);
        robot.rightFrontWheel.setPower(0);
        robot.rightBackWheel.setPower(0);
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }  // This strafes the robot for 1.3 seconds
        robot.leftFrontWheel.setPower(-STRAFE_SPEED);
        robot.leftBackWheel.setPower(STRAFE_SPEED);
        robot.rightFrontWheel.setPower(STRAFE_SPEED);
        robot.rightBackWheel.setPower(-STRAFE_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }
        // This stops the the robot from moving
        robot.leftFrontWheel.setPower(0);
        robot.leftBackWheel.setPower(0);
        robot.rightFrontWheel.setPower(0);
        robot.rightBackWheel.setPower(0);
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }
                // drives back until finds the wall!!
        while (opModeIsActive() && (rangeSensor.cmUltrasonic() >= 12)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
            robot.leftFrontWheel.setPower(-FORWARD_SPEED);
            robot.leftBackWheel.setPower(-FORWARD_SPEED);
            // This is the fine tuning to make the robot go completely straight
            robot.rightFrontWheel.setPower(-0.75*FORWARD_SPEED);
            robot.rightBackWheel.setPower(-0.75*FORWARD_SPEED);

        }
        // stops the robot
        robot.leftFrontWheel.setPower(0);
        robot.leftBackWheel.setPower(0);
        robot.rightFrontWheel.setPower(0);
        robot.rightBackWheel.setPower(0);
        Color.RGBToHSV(lineSensor.red() * 8, lineSensor.green() * 8, lineSensor.blue() * 8, lhsvValues);
        while (opModeIsActive() && (lhsvValues[0] <= 150)) {
            Color.RGBToHSV(lineSensor.red() * 8, lineSensor.green() * 8, lineSensor.blue() * 8, lhsvValues);
            Color.RGBToHSV(redBlueSensor.red() * 8, redBlueSensor.green() * 8, redBlueSensor.blue() * 8, rbhsvValues);
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addLine();
            telemetry.addData("lHue", lhsvValues[0]);
            telemetry.addData("lClear", lineSensor.alpha());
            telemetry.addData("lRed  ", lineSensor.red());
            telemetry.addData("lGreen", lineSensor.green());
            telemetry.addData("lBlue ", lineSensor.blue());
            telemetry.addData("rbClear", redBlueSensor.alpha());
            telemetry.addData("rbRed  ", redBlueSensor.red());
            telemetry.addData("rbGreen", redBlueSensor.green());
            telemetry.addData("rbBlue ", redBlueSensor.blue());
            telemetry.update();
            idle();

            robot.leftFrontWheel.setPower(FORWARD_SPEED);
            robot.leftBackWheel.setPower(-FORWARD_SPEED);
            robot.rightFrontWheel.setPower(-FORWARD_SPEED);
            robot.rightBackWheel.setPower(FORWARD_SPEED);

        }
        // stops the robot
        robot.leftFrontWheel.setPower(0);
        robot.leftBackWheel.setPower(0);
        robot.rightFrontWheel.setPower(0);
        robot.rightBackWheel.setPower(0);

    }


}
