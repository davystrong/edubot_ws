import rospy
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
import asyncio
from random import choice
import pandas as pd
import sys


setpoint = 0


async def prbs_writer(rate: float, limit=0.01, l_on=True, r_on=True):
    """
    Not strictly PRBS, just a random signal
    """
    pub_l = rospy.Publisher(
        '/edubot/wheel_L_effort_controller/command', Float64, queue_size=10)
    pub_r = rospy.Publisher(
        '/edubot/wheel_R_effort_controller/command', Float64, queue_size=10)

    async def write_prbs():
        global setpoint
        setpoint = choice([1, 0])*limit
        print(setpoint)
        if l_on:
            pub_l.publish(setpoint)
        if r_on:
            pub_r.publish(setpoint)

    while not rospy.is_shutdown():
        await asyncio.gather(write_prbs(), asyncio.sleep(rate))


async def speed_reader(rate: float, filename='data.csv', prbs_rate: float = 10, limit=0.01, l_on=True, r_on=True):
    l_speed = 0.0
    r_speed = 0.0
    last_time = 0.0
    last_prbs_time = 0.0
    setpoint = 0

    df = pd.DataFrame()

    def save_speed(data: JointState):
        nonlocal l_speed, r_speed
        l_speed = data.velocity[0]
        r_speed = data.velocity[1]

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/edubot/joint_states', JointState, save_speed)

    pub_l = rospy.Publisher(
        '/edubot/wheel_L_effort_controller/command', Float64, queue_size=10)
    pub_r = rospy.Publisher(
        '/edubot/wheel_R_effort_controller/command', Float64, queue_size=10)

    def write_prbs():
        nonlocal setpoint
        setpoint = choice([1, 0])*limit
        print(setpoint)
        if l_on:
            pub_l.publish(setpoint)
        if r_on:
            pub_r.publish(setpoint)

    def read_time(data: Clock):
        nonlocal df, last_time, last_prbs_time
        time = data.clock.secs + data.clock.nsecs*10**-9
        if time - last_time >= rate:
            last_time = (time//rate)*rate
            print(time)
            new_row = {'time': last_time, 'setpoint': setpoint,
                       'l_speed': l_speed, 'r_speed': r_speed}
            df = pd.concat([df, pd.DataFrame(new_row, index=[0])])
        if time - last_prbs_time >= prbs_rate:
            last_prbs_time = (time//prbs_rate)*prbs_rate
            write_prbs()

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/clock', Clock, read_time)

    while not rospy.is_shutdown():
        await asyncio.sleep(1)

    print('Saving data')
    df.to_csv(filename)


async def main():
    speed_reader_coro = speed_reader(0.1, prbs_rate=0.5, l_on=True, r_on=False)
    # prbs_writer_coro = prbs_writer(5, l_on=True, r_on=False)

    await asyncio.wait([speed_reader_coro], return_when=asyncio.FIRST_COMPLETED)


asyncio.run(main())
