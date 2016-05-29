#! /usr/bin/env python

from sensor_msgs.msg import LaserScan

from yamabros.topic import Topic


class Hokuyo(Topic):
    def __init__(self, topic='/scan', publisher_queue=100):
        Topic.__init__(self, topic, LaserScan, publisher_queue)

    def ranges(self):
        result = []
        def handler(data):
            result.append(data.ranges)

        self.subscribe(handler)
        while len(result) == 0:
            pass

        self.unsubscribe(handler)
        return result[0]
