# f1tenth_lab4

This assignment from [F1TENTH](https://f1tenth.org/learn.html) practices on reactive methods: follow the gap.

# How to run?
**Before cloning this respository**, make sure you have clone the [simulator](https://github.com/f1tenth/f1tenth_simulator) respository and set up the packages properly. The setup instructions can be found [here](https://f1tenth.readthedocs.io/en/stable/going_forward/simulator/sim_install.html).

> After you have clone this respository, run the script:

`roslaunch chee0134_lab4 chee0134_roslab.launch`

> Once the car is loaded into the Levine map in RViZ, press *n* to start navigation.

# Follow the Gap
My first result (*test.py*) could be seen in *IMG_8118.MOV* in my Google Drive. The result is not satisfactory because my method on choosing the furthest point in the maximum range of gap is not correct and not fine-tuned properly. The car moves very slowly as I prioritise finding the gap at the first attemept, instead of the velocity of the car.

On my second attempt (*reactive_gap_follow.py with the video IMG_8151.MOV*), the car is able to drive and turn properly at a high speed. I have completed **5 rounds of the track within 1 minute**. I changed the method on choosing the furthest point in the maximum range of gap from the array of the free spaces:
- If there is only one maximum value in the array of the gap, ex: np.array([3]), it will automically be selected to be the furthest point.
- If there are more than one same maximum values in the array, ex: np.array[(3,3,3)], the center of the maximum values will be used to choose the best point. By splitting the array of the free spaces into 3 parts, we can roughly determine the location of the gap by the location of center of the maximum values. If the center of the maximum values falls into the middle part, it will drive straight. Otherwise, it will turn.

# Limitations
- From line 168 in *reactive_gap_follow.py*, it potrays this script is more suitable to turn left than turn right. If it turns left, it will select the value on the right side of the center of the maximum values to have a smaller steering angle that considers the width of the car. If it turns right, the steering angle will be large, increasing higher risk of crashing into the wall.
- Based on the line above, due to the small steering angle, this script will not work if there is a narrow and short path after it turns around a corner. It takes time to steer back to zero degree after it turns. Therefore, parameters such as the maximum distance to be rejected, the best point within the gap and the velocity need to be tuned accordingly at different scenario.

## Videos
- As the size of the videos are too big, they are uploaded into my [Google Drive](https://drive.google.com/drive/folders/1-4xMJ_YMxJeRiX5eBb0VWjwdgGMpbtXY?usp=sharing).

