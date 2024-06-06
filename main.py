import asyncio
import logging
from enum import Enum
from typing import Union

import cv2
import RPi.GPIO as GPIO
import serial
from telegram import Update
from telegram.ext import (
    Application,
    ApplicationBuilder,
    CommandHandler,
    ContextTypes,
    MessageHandler,
    filters,
)

import utils
from ml import Classifier, Movenet

LOGGING_LEVEL = logging.INFO

# SET THESE FOR SERVO MOVEMENTS
CAM_X = 480
ANGLE_THRESHOLD = 10

# THESE WILL BE AUTOMATICALLY SET DURING `init`
ANGLE = 0
CHAT_ID = 0

# STATE RELATED STUFF
State = Enum("State", ["DEAD", "IDLE", "ACTIVE"])
STATE = State.DEAD
STATE_LOCK = asyncio.Lock()


# SET UP LOGGING
class CustomFormatter(logging.Formatter):
    grey = "\x1b[38;20m"
    yellow = "\x1b[33;20m"
    red = "\x1b[31;20m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"
    fmt = "[%(asctime)s] %(levelname)s - (%(funcName)s) %(message)s"

    FORMATS = {
        logging.DEBUG: grey + fmt + reset,
        logging.INFO: grey + fmt + reset,
        logging.WARNING: yellow + fmt + reset,
        logging.ERROR: red + fmt + reset,
        logging.CRITICAL: bold_red + fmt + reset,
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)


logging.basicConfig(level=logging.DEBUG)
root = logging.getLogger()

ch = logging.StreamHandler()
ch.setLevel(LOGGING_LEVEL)
ch.setFormatter(CustomFormatter())

root.addHandler(ch)
root.removeHandler(root.handlers[0])

# this section i copy directly from your code
GPIO.setmode(GPIO.BOARD)  # sets names to board mode, names pin according to nubers

GPIO.setup(3, GPIO.OUT)  # output to send pwm signal on
pwm = GPIO.PWM(3, 50)
pwm.start(0)


async def set_angle(angle: Union[int, float]) -> None:
    duty = angle / 18 + 2
    GPIO.output(3, True)  # turns on pin for output
    pwm.ChangeDutyCycle(duty)
    await asyncio.sleep(0.3)
    GPIO.output(3, False)  # turns off pin
    pwm.ChangeDutyCycle(0)  # not continuously sending inputs to the servo

    logging.info(f"servo angle set to {angle}, duty cycle {duty}")


async def ultrasensor() -> None:
    """Task that updates `STATE` based on the distance from the ultrasonic sensor.

    Reads data from serial port and sets `STATE` to ACTIVE if the distance is less than
    30 cm, and IDLE if the distance is greater than 30cm. It will only update `STATE`
    to ACTIVE if `STATE_LOCK` is not locked. This is to prevent the `STATE` from being
    updated to while the 10 seconds timeout from `stop` is still running.
    """

    global STATE

    # serial setup (copied from your code)
    ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)

    while True:
        logging.debug("requesting distance...")
        ser.reset_output_buffer()
        ser.reset_input_buffer()
        ser.write(b"R")  # send request for distance
        ser.flush()
        line = ser.readline()  # read response
        logging.info(f"received: {line}")

        if not line or line[-1] != 0x0A:  # check if last byte is LF (\n)
            logging.warning("invalid response, skipping...")
            continue
        else:
            line = line.decode("utf-8").strip()  # decode line

        try:
            distance = int(line)  # convert to distance int
            logging.info(f"distance: {distance} cm")
        except ValueError:
            logging.warning("could not decode distance '{line}', skipping...")
            continue

        # if distance is less than 30 cm and currently IDLE and not in 10 s timeout
        if distance < 30 and STATE == State.IDLE and not STATE_LOCK.locked():
            async with STATE_LOCK:
                STATE = State.ACTIVE  # set STATE to ACTIVE
            logging.info("STATE set to ACTIVE")

        # if distance is greater than 30 cm and currently ACTIVE
        elif distance > 30 and STATE == State.ACTIVE:
            async with STATE_LOCK:
                STATE = State.IDLE  # set STATE to IDLE
            logging.info("STATE set to IDLE")

        await asyncio.sleep(0)


async def cam(context: ContextTypes.DEFAULT_TYPE) -> None:
    """Task that processes images from camera if STATE is ACTIVE.

    Processing will only happen if STATE is ACTIVE. It will also only process images
    every 5 seconds. Firstly, it grabs the latest frame from the camera, which is sent
    to the pose detector. If the pose detector detects all keypoints of a person, the
    classifier is then used to classify the pose. The annotated image is then sent to
    the user, with the probabilities of each pose label. Any erorr will be sent to the
    user as well.
    """

    global ANGLE

    # set up pose detector and classifier
    pose_detector = Movenet("movenet_thunder")
    classifier = Classifier("new_pose_classifier", "new_pose_labels.txt")

    logging.info("pose detector and classifier initialized")

    # set up camera
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # set buffer size to 1 to always get latest

    logging.info("camera initialized")

    try:
        while True:
            # if STATE is not ACTIVE, do not do anything
            if not STATE == State.ACTIVE:
                logging.debug("STATE is not ACTIVE, skipping...")
                await asyncio.sleep(0)
                continue

            logging.debug("STATE is ACTIVE, processing image")

            """camera stuff"""
            # if camera is not opened, send error message to user and do nothing more
            if not cap.isOpened():
                logging.error("camera not opened, skipping...")
                await context.bot.send_message(
                    CHAT_ID,
                    "well i'm supposed to be taking images now but cam prolly dieded",
                )
                await asyncio.sleep(5)
                continue

            # grab latest frame from camera
            ret, frame = cap.read()

            # if not successful, send error message to user and do nothing more
            if not ret:
                logging.error("could not grab frame, skipping...")
                await context.bot.send_message(
                    CHAT_ID,
                    "well i'm supposed to be taking images now but cam prolly dieded",
                )
                await asyncio.sleep(5)
                continue

            # process image
            person = pose_detector.detect(frame)  # detect pose
            logging.info("pose detector ran")

            # if no person detected, send error message to user and do nothing more
            if not person:
                logging.warning("no person detected, skipping...")
                await context.bot.send_message(
                    CHAT_ID,
                    "well i'm supposed to be taking images now but no person detected",
                )
                await asyncio.sleep(5)
                continue

            """servo movement"""
            # find x-position of person and compare with camera center
            person_x = (
                person.bounding_box.end_point.x - person.bounding_box.start_point.x
            )
            logging.debug(f"person x-position: {person_x}")

            # if person is to the left of the camera, turn servo to the right
            if person_x < CAM_X - ANGLE_THRESHOLD:
                ANGLE += 5
                if ANGLE > 180:
                    ANGLE = 180
                    await context.bot.send_message(
                        CHAT_ID, "angle > 180, automatically set to 180 instead"
                    )

            # if person is to the right of the camera, turn servo to the left
            elif person_x > CAM_X + ANGLE_THRESHOLD:
                ANGLE -= 5
                if ANGLE < 0:
                    ANGLE = 0
                    await context.bot.send_message(
                        CHAT_ID, "angle < 0, automatically set to 0 instead"
                    )

            # set servo angle
            await set_angle(ANGLE)

            """pose classification"""
            # if any keypoints have a score less than 0.1, do not classify pose
            min_score = min([keypoint.score for keypoint in person.keypoints])
            if min_score < 0.1:
                prob_list = None
                logging.info("some keypoints have score < 0.1, skipping classification")
            else:
                prob_list = classifier.classify_pose(person)
                logging.info("pose classified")

            """send annotated image to user"""
            # annotate image
            image = utils.visualize(frame, [person])
            logging.debug("image annotated")

            # send annotated image to user, with caption of probabilities (or error if
            # `prob_list = None` which means some keypoints not detected)
            await context.bot.send_photo(
                CHAT_ID,
                cv2.imencode(".jpg", image)[1].tobytes(),
                "\n".join(
                    [f"{prob.label}: {prob.score}" for prob in prob_list]
                    if prob_list
                    else ["some keypoints not detected"]
                ),
            )
            logging.info(f"image sent to user, with {prob_list}")

            # only run image processing every 5 seconds
            await asyncio.sleep(5)

    # release camera if task is cancelled (bot stopped)
    except asyncio.CancelledError:
        cap.release()
        logging.info("camera released")


async def init(update: Update, context: ContextTypes.DEFAULT_TYPE):
    """Initialise the bot.

    This function initialises the bot by setting the global variables `STATE` to IDLE,
    `CHAT_ID` to the chat id of the user who sent the command, and servo `ANGLE` to 90.
    It then creates the tasks `ultrasensor` and `cam`. This would start the processing
    of the ultrasonic sensor and camera data.
    """

    global STATE
    global CHAT_ID
    global ANGLE

    # if STATE is not DEAD, do not initialise since bot is already initialised
    if not STATE == State.DEAD:
        if update.message.chat.id == CHAT_ID:
            await update.message.reply_text("already initialised by yourself")
            logging.info("already initialised by self")
        else:
            await update.message.reply_text(
                (
                    "already initialised by another user/chat: "
                    f"[{CHAT_ID}](tg://user?id={CHAT_ID}) (id will only work for user)"
                ),
                parse_mode="MarkdownV2",
            )
            logging.info("already initialised by user/chat")
        return

    async with STATE_LOCK:
        logging.info("initing")

        # set global variables
        STATE = State.IDLE
        CHAT_ID = update.message.chat.id
        ANGLE = 90

        # set initial servo angle
        await set_angle(ANGLE)

        # create tasks
        asyncio.create_task(ultrasensor())
        asyncio.create_task(cam(context))

        logging.info("init done")

    await update.message.reply_text("initialised")


async def stop(update: Update, _: ContextTypes.DEFAULT_TYPE):
    """Idle the bot.

    This function idles the bot by setting the global variable `STATE` to IDLE, which
    stops image processing until the ultrasonic sensor detects a distance less than
    30 cm again. There is a minimum idle time of 10 seconds, which is achieved through
    the use of the `STATE_LOCK` mutex lock.
    """

    # only allow user who initialised the bot to stop the bot
    if update.message.chat.id != CHAT_ID:
        await update.message.reply_text(
            "not authorised to stop bot as you did not initialise it"
        )
        logging.info("user authorised to stop bot")
        return

    # send message before idling
    await update.message.reply_text("idling...")
    async with STATE_LOCK:
        logging.info("idling")
        global STATE
        STATE = State.IDLE
        await asyncio.sleep(10)
        logging.info("idle done")


async def unknown(update: Update, _: ContextTypes.DEFAULT_TYPE):
    """Handle any unknown messages."""

    await update.message.reply_text("sorry dun understand pls try again")


async def post_shutdown(_: Application) -> None:
    """Post shutdown function.

    This function is called after the bot is stopped. It cancels all tasks, which
    ensures that the camera is released and all tasks are stopped.
    """

    logging.info("shutting down")

    for task in asyncio.all_tasks():
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

    logging.info("tasks cancelled, post shutdown done")


if __name__ == "__main__":
    # initialise bot
    application = (
        ApplicationBuilder()
        .token("5388299643:AAFrw1AboRY6rK0MChg3TqgnGe8-6ksYoZ4")
        .post_shutdown(post_shutdown)
        .build()
    )

    # make handlers for commands and messages
    init_handler = CommandHandler("init", init)
    stop_handler = CommandHandler("stop", stop)
    unknown_handler = MessageHandler(filters.TEXT, unknown)

    # add handlers to bot
    application.add_handler(init_handler)
    application.add_handler(stop_handler)
    application.add_handler(unknown_handler)

    # start bot
    application.run_polling()
