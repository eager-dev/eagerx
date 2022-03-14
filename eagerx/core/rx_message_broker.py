# IMPORT ROS
from typing import Any

import rospy
import rx.disposable
from rx import Observable, create
from rx.disposable import Disposable
from std_msgs.msg import UInt64, Bool

# IMPORT OTHER
from termcolor import cprint
import logging
import types
from functools import wraps
from threading import Condition

# IMPORT EAGERX
from eagerx.core.constants import DEBUG


def thread_safe_wrapper(func, condition):
    @wraps(func)
    def wrapped(*args, **kwargs):
        with condition:
            return func(*args, **kwargs)

    return wrapped


class RxMessageBroker(object):
    def __init__(self, owner):
        self.owner = owner

        # Determine log_level
        self.effective_log_level = logging.getLogger("rosout").getEffectiveLevel()

        # Ensure that we are not reading and writing at the same time.
        self.cond = Condition()

        # Structured as outputs[address][node_name] = {rx=Subject, node_name=node_name, source=RxOutput(...), etc..}
        self.rx_connectable = dict()

        # Structured as node_io[node_name][type][address] = {rx=Subject, disposable=rx_disposable, etc..}
        self.node_io = dict()
        self.disconnected = dict()
        self.connected_ros = dict()
        self.connected_rx = dict()

        # All publishers and subscribers (grouped to unregister when shutting down)
        self._publishers = []
        self.subscribers = []
        self.disposables = []

    # Every method is wrapped in a 'with Condition' block in order to be threadsafe
    def __getattribute__(self, name):
        attr = super(RxMessageBroker, self).__getattribute__(name)
        if isinstance(attr, types.MethodType):
            attr = thread_safe_wrapper(attr, self.cond)
        return attr

    def add_rx_objects(
        self,
        node_name,
        node=None,
        inputs=tuple(),
        outputs=tuple(),
        feedthrough=tuple(),
        state_inputs=tuple(),
        state_outputs=tuple(),
        targets=tuple(),
        node_inputs=tuple(),
        node_outputs=tuple(),
        reactive_proxy=tuple(),
        disposable: rx.disposable.CompositeDisposable = None,
    ):
        # Add disposable
        if disposable:
            self.disposables.append(disposable)

        # Determine tick address
        if node:
            ns = node.ns
        else:
            ns = self.node_io[node_name]["node"].ns
        tick_address = ns + "/bridge/outputs/tick"

        # Only add outputs that we would like to link with rx (i.e., skipping ROS (de)serialization)
        for i in outputs:
            if i["address"] == tick_address:
                continue
            assert i["address"] not in self.rx_connectable, (
                "Non-unique output (%s). All output names must be unique." % i["address"]
            )
            self.rx_connectable[i["address"]] = dict(rx=i["msg"], source=i, node_name=node_name, rate=i["rate"])

        # Register all I/O of node
        if node_name not in self.node_io:
            assert node is not None, 'No reference to Node "%s" was provided, during the first attempt to register it.'
            # Prepare io dictionaries
            self.node_io[node_name] = dict(
                node=node,
                inputs={},
                outputs={},
                feedthrough={},
                state_inputs={},
                state_outputs={},
                targets={},
                node_inputs={},
                node_outputs={},
                reactive_proxy={},
            )
            self.disconnected[node_name] = dict(inputs={}, feedthrough={}, state_inputs={}, targets={}, node_inputs={})
            self.connected_ros[node_name] = dict(inputs={}, feedthrough={}, state_inputs={}, targets={}, node_inputs={})
            self.connected_rx[node_name] = dict(inputs={}, feedthrough={}, state_inputs={}, targets={}, node_inputs={})
        n = dict(
            inputs={},
            outputs={},
            feedthrough={},
            state_inputs={},
            state_outputs={},
            targets={},
            node_inputs={},
            node_outputs={},
            reactive_proxy={},
        )
        for i in inputs:
            address = i["address"]
            cname_address = f"{i['name']}:{address}"
            self._assert_already_registered(cname_address, self.node_io[node_name], "inputs")
            self._assert_already_registered(cname_address, n, "inputs")
            n["inputs"][cname_address] = {
                "rx": i["msg"],
                "disposable": None,
                "source": i,
                "msg_type": i["msg_type"],
                "converter": i["converter"],
                "window": i["window"],
                "status": "disconnected",
            }
            n["inputs"][cname_address + "/reset"] = {
                "rx": i["reset"],
                "disposable": None,
                "source": i,
                "msg_type": UInt64,
                "status": "disconnected",
            }
        for i in outputs:
            address = i["address"]
            cname_address = f"{i['name']}:{address}"
            self._assert_already_registered(cname_address, self.node_io[node_name], "outputs")
            self._assert_already_registered(cname_address, n, "outputs")
            n["outputs"][cname_address] = {
                "rx": i["msg"],
                "disposable": None,
                "source": i,
                "msg_type": i["msg_type"],
                "rate": i["rate"],
                "converter": i["converter"],
                "status": "",
            }
            n["outputs"][cname_address + "/reset"] = {
                "rx": i["reset"],
                "disposable": None,
                "source": i,
                "msg_type": UInt64,
                "status": "",
            }

            # Create publisher
            i["msg_pub"] = rospy.Publisher(i["address"], i["msg_type"], queue_size=0, latch=True)
            d = i["msg"].subscribe(
                on_next=i["msg_pub"].publish,
                on_error=lambda e: print("Error : {0}".format(e)),
            )
            self.disposables.append(d)
            self._publishers.append(i["msg_pub"])
            i["reset_pub"] = rospy.Publisher(i["address"] + "/reset", UInt64, queue_size=0, latch=True)
            d = i["reset"].subscribe(
                on_next=i["reset_pub"].publish,
                on_error=lambda e: print("Error : {0}".format(e)),
            )
            self.disposables.append(d)
            self._publishers.append(i["reset_pub"])
        for i in feedthrough:
            address = i["address"]
            cname_address = f"{i['feedthrough_to']}:{address}"
            self._assert_already_registered(cname_address, self.node_io[node_name], "feedthrough")
            self._assert_already_registered(cname_address, n, "feedthrough")
            n["feedthrough"][cname_address] = {
                "rx": i["msg"],
                "disposable": None,
                "source": i,
                "msg_type": i["msg_type"],
                "converter": i["converter"],
                "window": i["window"],
                "status": "disconnected",
            }
            n["feedthrough"][cname_address + "/reset"] = {
                "rx": i["reset"],
                "disposable": None,
                "source": i,
                "msg_type": UInt64,
                "status": "disconnected",
            }
        for i in state_outputs:
            address = i["address"]
            cname_address = f"{i['name']}:{address}"
            self._assert_already_registered(cname_address, self.node_io[node_name], "state_outputs")
            self._assert_already_registered(cname_address, n, "state_outputs")
            n["state_outputs"][cname_address] = {
                "rx": i["msg"],
                "disposable": None,
                "source": i,
                "msg_type": i["msg_type"],
                "status": "",
            }
            if "converter" in i:
                n["state_outputs"][cname_address]["converter"] = i["converter"]

            # Create publisher
            i["msg_pub"] = rospy.Publisher(i["address"], i["msg_type"], queue_size=0, latch=True)
            d = i["msg"].subscribe(
                on_next=i["msg_pub"].publish,
                on_error=lambda e: print("Error : {0}".format(e)),
            )
            self.disposables.append(d)
            self._publishers.append(i["msg_pub"])
        for i in state_inputs:
            address = i["address"]
            try:
                cname_address = f"{i['name']}:{address}"
            except KeyError:
                cname_address = f"done_flag:{address}"
            if "msg" in i:  # Only true if sim state node (i.e. **not** for bridge done flags)
                self._assert_already_registered(cname_address + "/set", self.node_io[node_name], "state_inputs")
                self._assert_already_registered(cname_address + "/set", n, "state_inputs")
                n["state_inputs"][cname_address + "/set"] = {
                    "rx": i["msg"],
                    "disposable": None,
                    "source": i,
                    "msg_type": i["msg_type"],
                    "converter": i["converter"],
                    "status": "disconnected",
                }
            # Only true if **not** a real reset node (i.e., sim state & bridge done flag)
            if (cname_address + "/done") not in n["state_outputs"].keys():
                self._assert_already_registered(cname_address + "/done", self.node_io[node_name], "state_inputs")
                self._assert_already_registered(cname_address + "/done", n, "state_inputs")
                n["state_inputs"][cname_address + "/done"] = {
                    "rx": i["done"],
                    "disposable": None,
                    "source": i,
                    "msg_type": Bool,
                    "status": "disconnected",
                }
        for i in targets:
            address = i["address"]
            cname_address = f"{i['name']}:{address}"
            self._assert_already_registered(cname_address + "/set", self.node_io[node_name], "targets")
            self._assert_already_registered(cname_address + "/set", n, "targets")
            n["targets"][cname_address + "/set"] = {
                "rx": i["msg"],
                "disposable": None,
                "source": i,
                "msg_type": i["msg_type"],
                "converter": i["converter"],
                "status": "disconnected",
            }
        for i in node_inputs:
            address = i["address"]
            cname_address = f"{i['name']}:{address}"
            self._assert_already_registered(cname_address, self.node_io[node_name], "node_inputs")
            self._assert_already_registered(cname_address, n, "node_inputs")
            n["node_inputs"][cname_address] = {
                "rx": i["msg"],
                "disposable": None,
                "source": i,
                "msg_type": i["msg_type"],
                "status": "disconnected",
            }
        for i in node_outputs:
            address = i["address"]
            cname_address = f"{i['name']}:{address}"
            self._assert_already_registered(cname_address, self.node_io[node_name], "node_outputs")
            self._assert_already_registered(cname_address, n, "node_outputs")
            n["node_outputs"][cname_address] = {
                "rx": i["msg"],
                "disposable": None,
                "source": i,
                "msg_type": i["msg_type"],
                "status": "",
            }

            # Create publisher: (latched: register, node_reset, start_reset, reset, real_reset)
            i["msg_pub"] = rospy.Publisher(i["address"], i["msg_type"], queue_size=0, latch=True)
            d = i["msg"].subscribe(
                on_next=i["msg_pub"].publish,
                on_error=lambda e: print("Error : {0}".format(e)),
            )
            self.disposables.append(d)
            self._publishers.append(i["msg_pub"])
        for i in reactive_proxy:
            address = i["address"]
            cname_address = f"{i['name']}:{address}"
            self._assert_already_registered(cname_address, self.node_io[node_name], "reactive_proxy")
            self._assert_already_registered(cname_address, n, "reactive_proxy")
            n["reactive_proxy"][cname_address + "/reset"] = {
                "rx": i["reset"],
                "disposable": None,
                "source": i,
                "msg_type": UInt64,
                "rate": i["external_rate"],
                "status": "",
            }

            # Create publisher
            i["reset_pub"] = rospy.Publisher(i["address"] + "/reset", UInt64, queue_size=0, latch=True)
            d = i["reset"].subscribe(
                on_next=i["reset_pub"].publish,
                on_error=lambda e: print("Error : {0}".format(e)),
            )
            self.disposables.append(d)
            self._publishers.append(i["reset_pub"])

        # Add new addresses to already registered I/Os
        for key in n.keys():
            self.node_io[node_name][key].update(n[key])

        # Add new addresses to disconnected
        for key in ("inputs", "feedthrough", "state_inputs", "targets", "node_inputs"):
            self.disconnected[node_name][key].update(n[key].copy())

    def print_io_status(self, node_names=None):
        # Only print status for specific node
        if node_names is None:
            node_names = self.node_io.keys()
        else:
            if isinstance(node_names, str):
                node_names = [node_names]

        # Print status
        for node_name in node_names:
            cprint(
                ('OWNER "%s"' % self.owner).ljust(15, " ") + ('| OVERVIEW NODE "%s" ' % node_name).ljust(180, " "),
                attrs=["bold", "underline"],
            )
            for key in (
                "inputs",
                "feedthrough",
                "state_inputs",
                "targets",
                "node_inputs",
                "outputs",
                "state_outputs",
                "node_outputs",
                "reactive_proxy",
            ):
                if len(self.node_io[node_name][key]) == 0:
                    continue
                for cname_address in self.node_io[node_name][key].keys():
                    color = None
                    if key in (
                        "outputs",
                        "node_outputs",
                        "state_outputs",
                        "reactive_proxy",
                    ):
                        color = "cyan"
                    else:
                        if cname_address in self.disconnected[node_name][key]:
                            color = "red"
                        if cname_address in self.connected_rx[node_name][key]:
                            assert color is None, f"Duplicate connection status for address ({cname_address})."
                            color = "green"
                        if cname_address in self.connected_ros[node_name][key]:
                            assert color is None, f"Duplicate connection status for address ({cname_address})."
                            color = "blue"
                        assert (
                            color is not None
                        ), "Address (cname_address) not found in self.(disconnected, connected_rx, connected_ros)."
                    status = self.node_io[node_name][key][cname_address]["status"]

                    # Print status
                    entry = self.node_io[node_name][key][cname_address]
                    key_str = ("%s" % key).ljust(15, " ")
                    address_str = ("| %s " % cname_address).ljust(50, " ")
                    msg_type_str = ("| %s " % entry["msg_type"].__name__).ljust(10, " ")
                    if "converter" in entry:
                        converter_str = ("| %s " % entry["converter"].__class__.__name__).ljust(23, " ")
                    else:
                        converter_str = ("| %s " % "").ljust(23, " ")
                    if "window" in entry:
                        window_str = ("| %s " % entry["window"]).ljust(8, " ")
                    else:
                        window_str = ("| %s " % "").ljust(8, " ")
                    if "rate" in entry:
                        rate_str = "|" + ("%s" % entry["rate"]).center(3, " ")
                    else:
                        rate_str = "|" + "".center(3, " ")
                    status_str = ("| %s" % status).ljust(60, " ")

                    log_msg = key_str + rate_str + address_str + msg_type_str + converter_str + window_str + status_str
                    cprint(log_msg, color)
            print(" ".center(140, " "))

    def connect_io(self, print_status=True):
        # If log_level is not high enough, overwrite print_status
        if self.effective_log_level > DEBUG:
            print_status = False

        for node_name, node in self.disconnected.items():
            # Skip if no disconnected addresses
            num_disconnected = 0

            for _key, addresses in node.items():
                num_disconnected += len(addresses)
            if num_disconnected == 0:
                continue

            # Else, initialize connection
            print_status and cprint(
                ('OWNER "%s"' % self.owner).ljust(15, " ") + ('| CONNECTING NODE "%s" ' % node_name).ljust(180, " "),
                attrs=["bold", "underline"],
            )
            for key, addresses in node.items():
                for cname_address in list(addresses.keys()):
                    _, address = self._split_cname_address(cname_address)
                    entry = addresses[cname_address]
                    assert (
                        cname_address not in self.connected_rx[node_name][key]
                    ), f"Address ({cname_address}) of this node ({node_name}) already connected via rx."
                    assert (
                        cname_address not in self.connected_ros[node_name][key]
                    ), f"Address ({cname_address}) of this node ({node_name}) already connected via ROS."
                    if address in self.rx_connectable.keys():
                        color = "green"
                        status = "Rx".ljust(4, " ")
                        entry["rate"] = self.rx_connectable[address]["rate"]
                        rate_str = f"|{str(entry['rate']).center(3, ' ')}"
                        node_str = f'| {self.rx_connectable[address]["node_name"].ljust(40, " ")}'
                        msg_type_str = f'| {self.rx_connectable[address]["source"]["msg_type"].__name__}'.ljust(12, " ")
                        converter_str = f'| {self.rx_connectable[address]["source"]["converter"].__class__.__name__}'.ljust(
                            12, " "
                        )
                        status += node_str + msg_type_str + converter_str
                        self.connected_rx[node_name][key][cname_address] = entry
                        T = self.rx_connectable[address]["rx"]
                    else:
                        color = "blue"
                        status = "ROS |".ljust(5, " ")
                        rate_str = "|" + "".center(3, " ")
                        msg_type = entry["msg_type"]
                        self.connected_ros[node_name][key][cname_address] = entry
                        T = from_topic(msg_type, address, node_name, self.subscribers)

                    # Subscribe and change status
                    entry["disposable"] = T.subscribe(entry["rx"])
                    self.disposables.append(entry["disposable"])
                    entry["status"] = status

                    # Print status
                    key_str = ("%s" % key).ljust(15, " ")
                    address_str = ("| %s " % cname_address).ljust(50, " ")
                    msg_type_str = ("| %s " % entry["msg_type"].__name__).ljust(10, " ")
                    status_str = ("| Connected via %s" % status).ljust(60, " ")

                    if "converter" in entry:
                        converter_str = ("| %s " % entry["converter"].__class__.__name__).ljust(23, " ")
                    else:
                        converter_str = ("| %s " % "").ljust(23, " ")
                    if "window" in entry:
                        window_str = ("| %s " % entry["window"]).ljust(8, " ")
                    else:
                        window_str = ("| %s " % "").ljust(8, " ")

                    log_msg = key_str + rate_str + address_str + msg_type_str + converter_str + window_str + status_str
                    print_status and cprint(log_msg, color)

                    # Remove address from disconnected
                    addresses.pop(cname_address)

            print_status and print("".center(140, " "))

    def _split_cname_address(self, cname_address):
        res = cname_address.split(":")
        if len(res) == 2:
            cname, address = res
        else:
            cname, address = None, res[0]
        return cname, address

    def _assert_already_registered(self, name, d, component):
        assert name not in d[component], f'Cannot re-register the same address ({name}) twice as "{component}".'

    def shutdown(self):
        rospy.logdebug(f"[{self.owner}] RxMessageBroker.shutdown() called.")
        [pub.unregister() for pub in self._publishers]
        [sub.unregister() for sub in self.subscribers]
        [d.dispose() for d in self.disposables]


def from_topic(topic_type: Any, topic_name: str, node_name, subscribers: list) -> Observable:
    def _subscribe(observer, scheduler=None) -> Disposable:
        try:
            wrapped_sub = []

            def cb_from_topic(msg, wrapped_sub):
                try:
                    observer.on_next(msg)
                except rospy.exceptions.ROSException as e:
                    sub = wrapped_sub[0]
                    sub.unregister()
                    rospy.logdebug(f"[{sub.name}]: Unregistered this subscription because of exception: {e}")

            sub = rospy.Subscriber(topic_name, topic_type, callback=cb_from_topic, callback_args=wrapped_sub)
            wrapped_sub.append(sub)
            subscribers.append(sub)
        except Exception as e:
            rospy.logwarn("[%s]: %s" % (node_name, e))
            raise
        return observer

    return create(_subscribe)
