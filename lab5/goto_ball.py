#!/usr/bin/env python3

import asyncio
import sys

import cv2
import numpy as np

sys.path.insert(0, '../lab4')
import find_ball

import cozmo

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')


# Define a decorator as a subclass of Annotator; displays battery voltage
class BatteryAnnotator(cozmo.annotate.Annotator):
    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)
        batt = self.world.robot.battery_voltage
        text = cozmo.annotate.ImageText('BATT %.1fv' % batt, color='green')
        text.render(d, bounds)

# Define a decorator as a subclass of Annotator; displays the ball
class BallAnnotator(cozmo.annotate.Annotator):

    ball = None

    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)

        if BallAnnotator.ball is not None:

            #double size of bounding box to match size of rendered image
            BallAnnotator.ball = np.multiply(BallAnnotator.ball,2)

            #define and display bounding box with params:
            #msg.img_topLeft_x, msg.img_topLeft_y, msg.img_width, msg.img_height
            box = cozmo.util.ImageBox(BallAnnotator.ball[0]-BallAnnotator.ball[2],
                                      BallAnnotator.ball[1]-BallAnnotator.ball[2],
                                      BallAnnotator.ball[2]*2, BallAnnotator.ball[2]*2)
            goal = str(self.world.robot.goal)
            cozmo.annotate.add_img_box_to_image(image, box, "green", text=cozmo.annotate.ImageText(goal))

            BallAnnotator.ball = None


async def run(robot: cozmo.robot.Robot):
    '''The run method runs once the Cozmo SDK is connected.'''

    #add annotators for battery level and ball bounding box
    robot.world.image_annotator.add_annotator('battery', BatteryAnnotator)
    robot.world.image_annotator.add_annotator('ball', BallAnnotator)


    try:
        state = 0
        def turn_to(curr, goal):
            return (curr + goal) / 2

        while True:
            #get camera image
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

            #convert camera image to opencv format
            opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)

            #find the ball
            ball = find_ball.find_ball(opencv_image)

            #set annotator ball
            BallAnnotator.ball = ball

            goal = [0,0]
            limit = 20
            ball_state_weight = 3
            thresh = .2 * limit
            dist_thresh = event.image.width / 2


            if ball is None:
                state = max(state - 1, 0)
            else:
                state = min(state + ball_state_weight, limit)
                w = event.image.width
                unweighted = [w + ball[0], 2*w - ball[0]]
                goal       = np.multiply(unweighted, 30 / (2*w))
                robot.goal = ball
                if ball[2] > dist_thresh:
                    robot.move_lift(5)
                else:
                    robot.move_lift(-5)

            if state <= thresh:
                motors = np.multiply([10, -10], 1 - (state/thresh))
            else:
                motors = np.multiply(goal, (state - thresh)/(limit - thresh))

            await robot.drive_wheels(*motors)


    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)



if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)

