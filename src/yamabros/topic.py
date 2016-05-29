#! /usr/bin/env python

from rospy import resolve_name, Publisher, Subscriber

class Topic(object):
    def __init__(self, name, message, publisher_queue=100):
        self.message = message
        self.name = resolve_name(name)
        self.publisher = Publisher(self.name, self.message, queue_size=publisher_queue)
        self.subscribers = {}

    def __del__(self):
        self.close()

    def close(self):
        self.publisher.unregister()
        for (handler, subscriber) in self.subscribers.items():
            subscriber.unregister()

    def publish(self, message):
        self.publisher.publish(message)

    def subscribe(self, handler):
        self.subscribers[handler] = Subscriber(self.name, self.message, handler)

    def unsubscribe(self, handler):
        del self.subscribers[handler]
