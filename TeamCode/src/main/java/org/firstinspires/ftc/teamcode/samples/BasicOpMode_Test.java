/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: TEST", group="Linear Opmode")
@Disabled
public class BasicOpMode_Test extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorleft1 = null;
    private DcMotor motorright1 = null;
    private DcMotor motorleft2 = null;
    private DcMotor motorright2 = null;
    private Servo servoIntake1 = null;
    private Servo servoIntake2 = null;
    private Servo servocarosuel = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorleft1  = hardwareMap.get(DcMotor.class, "motorleft1");
        motorright1 = hardwareMap.get(DcMotor.class, "motorright1");
        motorleft2  = hardwareMap.get(DcMotor.class, "motorleft2");
        motorright2 = hardwareMap.get(DcMotor.class, "motorright2");
        servoIntake1  = hardwareMap.get(Servo.class, "servoIntake1");
        servoIntake2  = hardwareMap.get(Servo.class, "servoIntake2");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorleft1.setDirection(DcMotor.Direction.FORWARD);
        motorright1.setDirection(DcMotor.Direction.FORWARD);
        motorleft2.setDirection(DcMotor.Direction.REVERSE);
        motorright2.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double drivePowerR;
            double drivePowerL;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double driveL = -gamepad1.left_stick_y;
            //double turn  =  gamepad1.right_stick_x;
            drivePowerL    = Range.clip(driveL, -1.0, 1.0) ;

            double driveR = -gamepad1.right_stick_y;
            //double turn  =  gamepad1.right_stick_y;
            drivePowerR    = Range.clip(driveR, -1.0, 1.0) ;

            if(gamepad2.y) {
                // move to 0 degrees.
                servoIntake1.setPosition(0);
            } else if (gamepad2.x || gamepad2.b) {
                // move to 90 degrees.
                servoIntake1.setPosition(0.5);
            } else if (gamepad2.a) {
                // move to 180 degrees.
                servoIntake1.setPosition(1);
            }
            if(gamepad2.y) {
                // move to 0 degrees.
                servoIntake2.setPosition(0);
            } else if (gamepad2.x || gamepad2.b) {
                // move to 90 degrees.
                servoIntake2.setPosition(0.5);

            } else if (gamepad2.x || gamepad2.b) {
                // move to 90 degrees.
                servoIntake2.setPosition(0.5);
            } else if (gamepad2.a) {
                // move to 180 degrees.
                servoIntake2.setPosition(1);
            }


            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            motorleft1.setPower(drivePowerL);
            motorright1.setPower(drivePowerR);
            motorleft2.setPower(drivePowerL);
            motorright2.setPower(drivePowerR);

            // Show the elapsed game time and wheel power.
            //telemetry.addData("Servo Position", servoIntake1.getPosition());
            //telemetry.addData("Servo Position", servoIntake2.getPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("MotorL", "drive (%.2f)", drivePowerL);
            telemetry.addData("MotorR", "drive (%.2f)", drivePowerR);
            telemetry.update();
        }
    }
}
