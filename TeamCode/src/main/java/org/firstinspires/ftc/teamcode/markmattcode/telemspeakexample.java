package org.firstinspires.ftc.teamcode.markmattcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name = "BRAD PLEASE RUN THIS AND WAIT A MINUTE", group = "!!Test")
public class telemspeakexample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.speak("Hardware Status,  Initialized");
        waitForStart();
        while(opModeIsActive())
        {
            //telemetry.speak(you ve asked me to speak have you, 1 2 3 4 5 6 7 8 9 10, 1000, 10000, 100000, 1000000, 100000000000, 7456562, integer, towersize, odometry, system check, robot status scanning, sampling, skystone is on the right, moving plate, parking);
            telemetry.speak("Ground Control to Major Tom Ground Control to Major Tom Take your protein pills and put your helmet on Ground Control to Major Tom (ten, nine, eight, seven, six) Commencing countdown, engines on (five, four, three) Check ignition and may God's love be with you (two, one, liftoff) This is Ground Control to Major Tom You ve really made the grade And the papers want to know whose shirts you wear Now it s time to leave the capsule if you dareee This is Major Tom to Ground Control I'm stepping through the doooooor And I'm floating in a most peculiar way And the stars look very differenttt today For here Am I sitting in a tin can Far above the world Planet Earth is blue And there s nothing I can do Though I'm past one hundred thousand miles I'm feeling very still And I think my spaceship knows which way to go Tell my wife I love her very much she knows Ground Control to Major Tom Your circuit s dead, there s something wrong Can you hear me, Major Tom? Can you hear me, Major Tom? Can you hear me, Major Tom? Can you Here am I floating  round my tin can Far above the moon Planet Earth is blue And there s nothing I can do");
            sleep(60000);
            telemetry.speak("Domo arigato, Mr. Roboto Mata au himade  Domo arigato, Mr. Roboto [どうもありがとうミスターロボット], Himitsu wo shiri tai [秘密を知りたい]Youre wondering who I am (secret secret Ive got a secret)Machine or mannequin (secret secret I've got a secret)With parts made in Japan (secret secret I've got a secret) I am the modern man I've got a secret I've been hiding under my skin My heart is human, my blood is boiling, my brain I.B.M. So if you see me acting strangely, don't be surprised I'm just a man who needed someone, and somewhere to hide To keep me alive, just keep me alive Somewhere to hide, to keep me alive I'm not a robot without emotions. I'm not what you see I've come to help you with your problems, so we can be free I'm not a hero, I'm not the savior, forget what you know I'm just a man whose circumstances went beyond his control Beyond my control. We all need control I need control. We all need control I am the modern man (secret secret I've got a secret) Who hides behind a mask (secret secret I've got a secret) So no one else can see (secret secret I've got a secret)My true identity Domo arigato, Mr. Roboto, domo...domo Domo arigato, Mr. Roboto, domo...domo Domo arigato, Mr. Roboto Domo arigato, Mr. Roboto Domo arigato, Mr. Roboto Domo arigato, Mr. Roboto Thank you very much, Mr. Roboto For doing the jobs that nobody wants to And thank you very much, Mr. Roboto For helping me escape just when I needed to Thank you, thank you, thank you I want to thank you, please, thank you The problem's plain to see: Too much technology Machines to save our lives Machines dehumanize The time has come at last (secret secret I've got a secret)To throw away this mask (secret secret I've got a secret) Now everyone can see (secret secret I've got a secret)My true identity...I'm Kilroy! Kilroy!.....                  Kilroy!.....          Kilroy!......");
            sleep(2000000000);
        }
    }
}
