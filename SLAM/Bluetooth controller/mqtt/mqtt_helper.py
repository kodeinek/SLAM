# python3.6
# mosquitto_subh 82.145.72.202 - -t "anafi_visual_feedback" -u user2 -P user2 -p 1883
import random
import json
import time

from paho.mqtt import client as mqtt_client
import numpy as np
import threading
import argparse

print('ok')


# # global out
# # broker = 'broker.emqx.io'
# # broker = 'localhost'
# broker = '82.145.73.202'  # 192.168.250.55'
# # broker = '192.168.250.53'
# port = 1883
# topic = "anafi_visual_feedback"
# # topic = "python/mqtt"
# # generate client ID with pub prefix randomly
# client_id = f'python-mqtt-{random.randint(0, 100)}'
# username = 'user2'
# password = 'user2'


# password = 'user1'
# 120421ltnet

class MqttVisualeFeedback(threading.Thread):
    def __init__(self, broker='82.145.73.202', port=1883, topic="anafi_visual_feedback",
                 username='user2', password='user2'):
        super().__init__()
        self.broker = broker
        self.port = port
        self.topic = topic
        self.username = username
        self.password = password
        self.client_id = f'python-mqtt-{random.randint(0, 100)}'
        self.client = None
        # self.connect_mqtt()
        # self.subscribe()
        self.log = np.zeros((100000, 5))
        # self.log = np.zeros((100000, 7))
        self.i = 0
        pass

    def connect_mqtt(self, ) -> mqtt_client:
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print(f"Connected to MQTT Broker! {self.broker=}, {self.topic=}, {self.username=}")
            else:
                print("Failed to connect, return code %d\n", rc, self.topic)

        client = mqtt_client.Client(self.client_id)
        client.username_pw_set(self.username, self.password)
        client.on_connect = on_connect
        client.connect(self.broker, self.port)
        self.client = client

    def subscribe(self):
        mqtt_client = self.client

        def on_message(client, userdata, msg):
            # global out
            str = bytes(list(msg.payload)).decode('utf-8')
            # print((bytes(list(msg.payload)).decode('utf-8')))
            out = json.loads(str)
            # for i in out:
            #     print(i)
            # print(list(msg.payload))
            # w = ''.join(chr(i) for i in list(msg.payload))
            # print(f"Received `{w}` from `{msg.topic}` topic")
            #print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
            # print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
            self.log[self.i, :] = np.array(out)
            self.i += 1

        self.client.subscribe(self.topic)
        self.client.on_message = on_message

    def run(self):
        # global out
        self.connect_mqtt()
        self.subscribe()
        # self.subscribe(self.client)
        # client = connect_mqtt()
        # subscribe(client)
        self.client.loop_forever()


    def get_last(self):
        return self.log[self.i - 1, :]


class MqttFeedback(threading.Thread):
    def __init__(self, broker='82.145.73.202', port=1883, topic="anafi_control",
                 username='user2', password='user2'):
        super().__init__()
        self.broker = broker
        self.port = port
        self.topic = topic
        self.username = username
        self.password = password
        self.client_id = f'python-mqtt-{random.randint(0, 100)}'
        self.client = None
        #self.connect_mqtt()
        #self.subscribe()
        self.log = [self.parse_cmd("-mode a")]  #np.zeros((100000, 7))
        self.i = 0
        #pass

    def connect_mqtt(self, ) -> mqtt_client:
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print(f"Connected to MQTT Broker! {self.broker=}, {self.topic=}, {self.username=}")
            else:
                print("Failed to connect, return code %d\n", rc, self.topic)

        client = mqtt_client.Client(self.client_id)
        client.username_pw_set(self.username, self.password)
        client.on_connect = on_connect
        client.connect(self.broker, self.port)
        self.client = client

    def parse_cmd(self, cmdstr):
        parser = argparse.ArgumentParser(cmdstr, description='Argparse Test script')
        parser.add_argument("-mode", help='manual set point', choices=["a", 'm', 'l', 's'], default='s')
        #parser.add_argument("-a", help='some parameter', default=[1.5,1, 1.4, 0],)
        parser.add_argument("-Kroll", type=float, default=1.0, help='roll gain')
        parser.add_argument("-Kpitch", type=float, default=1.0, help='roll gain')
        parser.add_argument("-Kyaw", type=float, default=1.0, help='roll gain')
        parser.add_argument("-Kz", type=float, default=1.0, help='roll gain')

        #parser.add_argument('integers2', metavar='M', type=int,  default=[30,30,30,30])

        cmds = cmdstr.split()
        args = parser.parse_args(cmds)
        print(args)
        return args

    def subscribe(self):
        mqtt_client = self.client

        def on_message(client, userdata, msg):
            # global out
            str = bytes(list(msg.payload)).decode('utf-8')
            #print((bytes(list(msg.payload)).decode('utf-8')))
            #out = json.loads(str)
            out = self.parse_cmd(str)
            print(str)
            # print(out)
            # for i in out:
            #     print(i)
            # print(list(msg.payload))
            # w = ''.join(chr(i) for i in list(msg.payload))
            # print(f"Received `{w}` from `{msg.topic}` topic")
            #print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
            print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
            self.log.append(out)
            self.i += 1

        self.client.subscribe(self.topic)
        self.client.on_message = on_message

    def run(self):
        # global out
        self.connect_mqtt()
        self.subscribe()
        # self.subscribe(self.client)
        # client = connect_mqtt()
        # subscribe(client)
        self.client.loop_forever()

    def get_last(self):
        #print(self.i, self.log)
        return self.log[-1]


if __name__ == '__main__':
    # m = MqttVisualeFeedback()
    m = MqttFeedback()

    #m.run()
    m.start()
    time.sleep(1)
    print(m.i)
    print(m.get_last())
