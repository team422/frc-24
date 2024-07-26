import ntcore
import argparse
import os
from os.path import basename
import logging
import time

inst = ntcore.NetworkTableInstance.getDefault()

identity = f"{basename(__file__)}-{os.getpid()}"
inst.startClient4(identity)

inst.setServer("127.0.0.1")

class PoseSubscriber:
    def __init__(self, pose):
        self.pose = pose
        self.subscriber = inst.getTable("pose").getDoubleArrayTopic("pose").subscribe([-2.0], ntcore.PubSubOptions(pollStorage=10))
        while True:
            print("---", ntcore._now())
            print("/data/1:", self.subscriber.readQueue())
            print("/data/2:", self.subscriber.readQueue())

            # time.sleep(1.2)




    def valueChanged(self, data):
        self.pose = data
        print(self.pose)
    def close(self):
        pass


sub = PoseSubscriber(0)