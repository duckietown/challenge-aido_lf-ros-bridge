import logging
import time
from queue import Empty, Queue

import numpy as np
from aido_schemas import (
    Duckiebot1Commands,
    Duckiebot1Observations,
    EpisodeStart,
    GetCommands,
    LEDSCommands,
    protocol_agent_duckiebot1,
    PWMCommands,
    RGB,
)
from zuper_nodes_wrapper import Context, wrap_direct

from duckietown_aido_ros_bridge.commons import wrap_for_errors


class AIDOAgent:
    def __init__(self, qcommands: Queue, qimages: Queue, logger):
        self.qcommands = qcommands
        self.qimages = qimages
        self.logger = logger

    def init(self, context: Context):
        context.info("init()")

    def on_received_seed(self, context: Context, data: int):
        np.random.seed(data)

    def on_received_episode_start(self, context: Context, data: EpisodeStart):
        self.logger.info("Starting episode %s." % data)
        # TODO should we reset things?

    def on_received_observations(self, context: Context, data: Duckiebot1Observations):
        self.logger.info("Received observations")
        self.qimages.put(data)

    def on_received_get_commands(self, context: Context, data: GetCommands):
        self.logger.info("Received request for GetCommands")
        MAX_WAIT = 5
        try:
            action = self.qcommands.get(block=True, timeout=MAX_WAIT)
        except Empty:
            msg = f"Received no commands for {MAX_WAIT}s. Bailing out "
            raise Exception(msg) from None

        pwm_left, pwm_right = action

        grey = RGB(0.5, 0.5, 0.5)
        led_commands = LEDSCommands(grey, grey, grey, grey, grey)
        pwm_commands = PWMCommands(motor_left=pwm_left, motor_right=pwm_right)
        commands = Duckiebot1Commands(pwm_commands, led_commands)
        context.write("commands", commands)

    def finish(self, context):
        self.logger.info("finish()")


@wrap_for_errors
def run_agent(q_control, q_images, q_commands):
    logging.basicConfig()
    logger = logging.getLogger("run_agent")
    logger.setLevel(logging.DEBUG)

    agent = AIDOAgent(q_images, q_commands, logger)
    wrap_direct(agent, protocol_agent_duckiebot1, args=[])
    logger.info("run_agent ended")
    time.sleep(1000)
