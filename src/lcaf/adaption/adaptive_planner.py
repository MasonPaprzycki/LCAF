from __future__ import annotations

from copy import deepcopy
from threading import Lock

from lcaf.utils.toolpath import ToolpathOperation

class AdaptivePlanner:
    """
    Runs asynchronous analysis using telemetry.

    ForgeBrain only interacts through

        adapt()

    which returns a new execution queue.
    """

    def __init__(self):

        self.telemetry = None

        self.latest_snapshot = None

        self.lock = Lock()

    #
    # Called asynchronously whenever telemetry is published.
    #
    def update(self, telemetry):

        with self.lock:

            self.latest_snapshot = telemetry

            #
            # Future:
            #
            # update thermal model
            # update FEM
            # update MPM
            # update digital twin
            # estimate wear
            # estimate billet temperature
            #

    #
    # Called synchronously from ForgeBrain.
    #
    def adapt(self, queue):

        with self.lock:

            #
            # Work on a copy.
            #
            new_queue = deepcopy(queue)

            #
            # Future adaptation algorithms modify
            # new_queue.operations here.
            #

            return new_queue
        
        