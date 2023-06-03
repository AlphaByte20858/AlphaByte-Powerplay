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

package org.firstinspires.ftc.teamcode.PID;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Drive By Encoder", group="Robot")
@Disabled
public class RobotAutoDriveByEncoder_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor MEF, MET, MDF, MDT = null;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_CM           = 2.54  * ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415));


    @Override
    public void runOpMode() {

        MEF  = hardwareMap.get(DcMotor.class, "LeftDriveUp");
        MDF  = hardwareMap.get(DcMotor.class, "RightDriveUp");
        MET = hardwareMap.get(DcMotor.class, "LeftDriveDown");
        MDT = hardwareMap.get(DcMotor.class, "RightDriveDown");

        MEF.setDirection(DcMotor.Direction.FORWARD);
        MDF.setDirection(DcMotor.Direction.FORWARD);
        MET.setDirection(DcMotor.Direction.REVERSE);
        MDT.setDirection(DcMotor.Direction.REVERSE);

        modemoto(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        modemoto(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double DF, double DT, double EF, double ET) {
        int newDFTarget;
        int newEFTarget;
        int newDTTarget;
        int newETTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newDFTarget = MDF.getCurrentPosition() + (int)(DF * COUNTS_PER_CM);
            newEFTarget = MEF.getCurrentPosition() + (int)(EF * COUNTS_PER_CM);
            newDTTarget = MDT.getCurrentPosition() + (int)(DT * COUNTS_PER_CM);
            newETTarget = MET.getCurrentPosition() + (int)(ET * COUNTS_PER_CM);

            MDF.setTargetPosition(newDFTarget);
            MEF.setTargetPosition(newEFTarget);
            MDT.setTargetPosition(newDTTarget);
            MET.setTargetPosition(newETTarget);

            // Turn On RUN_TO_POSITION
            modemoto(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            MDF.setPower(Math.abs(speed));
            MEF.setPower(Math.abs(speed));
            MDT.setPower(Math.abs(speed));
            MET.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (MDF.isBusy() && MEF.isBusy() && MDT.isBusy() && MET.isBusy())) {
                idle();
            }

            // Stop all motion;
            MDF.setPower(0);
            MDT.setPower(0);
            MEF.setPower(0);
            MET.setPower(0);
            // Turn off RUN_TO_POSITION
            modemoto(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    public void modemoto(DcMotor.RunMode mode){
        MDF.setMode(mode);
        MDT.setMode(mode);
        MEF.setMode(mode);
        MET.setMode(mode);
    }
}