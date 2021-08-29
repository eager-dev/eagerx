#!/usr/bin/env python3
import rospy
import threading
import time
from std_msgs.msg import Bool
from collections import deque

class Input(object):
    def __init__(self, topic, msg_class, input_rate):
        super(Input, self).__init__()

        self.input_dt = 1.0/input_rate

        self.expected_input = 1 #What to init to?
        self.condition = threading.Condition()
        self.callback_queue = deque()
        self.current_data = None

        self.sub = rospy.Subscriber(topic, msg_class, callback=self._callback)

    def _callback(self, msg):
        with self.condition:
            self.callback_queue.append(msg.data)
            self.condition.notify_all()

    def get_inputs_in_step(self, time, dt):

        time_since_input = (time % self.input_dt) + dt
        return int(time_since_input / self.input_dt)

    def get(self, time, dt):
        self.expected_input = self.get_inputs_in_step(time, dt)
        print("expected:", self.expected_input)

        if self.expected_input == 0:
            return [self.current_data]

        with self.condition:
            while len(self.callback_queue) < self.expected_input:
                print('expecting more data')
                if not self.condition.wait(10):
                    raise Exception("Did get data after 10 seconds!")
            print('got data!')
            output = [self.callback_queue.pop() for _ in range(self.expected_input)]
            self.current_data = output[-1]
            return output

class Comms(object):

    def __init__(self, node_rate, inputs):
        super(Comms, self).__init__()

        self.time = 0
        self.dt = 1.0/node_rate
        self.inputs = inputs

    def run(self):
        t = time.time()
        while not rospy.is_shutdown():
            for input in self.inputs:
                data = input.get(self.time, self.dt)
                print(data)
            self.time += self.dt
            print(time.time()-t)
            t = time.time()


if __name__ == "__main__":
    rospy.init_node("comms")

    node_rate = 2 # Hz
    input = Input("test", Bool, 1)

    comm = Comms(node_rate, [input])
    comm.run()

    