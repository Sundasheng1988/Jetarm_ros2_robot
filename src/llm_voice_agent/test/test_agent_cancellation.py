#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import importlib
import os
import sys
import types
import unittest
from unittest import mock

from llm_voice_agent.runtime_control import CancellationTokenSlot
from llm_voice_agent.voice_backends import VoiceBackendCancelled


def _load_agent_without_ros():
    """Import the node class with inert ROS message/type stubs."""

    class _Node:
        pass

    class _Executor:
        pass

    class _CallbackGroup:
        pass

    class _Message:
        def __init__(self, data=None):
            self.data = data

    class _EnvObjectArray:
        pass

    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = _Node
    rclpy_executors = types.ModuleType("rclpy.executors")
    rclpy_executors.MultiThreadedExecutor = _Executor
    rclpy_groups = types.ModuleType("rclpy.callback_groups")
    rclpy_groups.ReentrantCallbackGroup = _CallbackGroup
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = _Message
    std_msgs_msg.Bool = _Message
    robot_interfaces = types.ModuleType("robot_interfaces")
    robot_interfaces_msg = types.ModuleType("robot_interfaces.msg")
    robot_interfaces_msg.EnvObjectArray = _EnvObjectArray

    stubs = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "rclpy.executors": rclpy_executors,
        "rclpy.callback_groups": rclpy_groups,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "robot_interfaces": robot_interfaces,
        "robot_interfaces.msg": robot_interfaces_msg,
    }
    sys.modules.pop("llm_voice_agent.llm_voice_agent_node", None)
    with mock.patch.dict(sys.modules, stubs):
        return importlib.import_module("llm_voice_agent.llm_voice_agent_node")


agent_module = _load_agent_without_ros()


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(("info", message))

    def warning(self, message):
        self.messages.append(("warning", message))


def _agent_for_request():
    agent = agent_module.LlmVoiceAgent.__new__(agent_module.LlmVoiceAgent)
    agent.llm_connect_timeout_s = 1.0
    agent.llm_timeout_s = 2.0
    agent.llm_backend = "glm"
    agent.llm_fallback_backend = "ollama"
    agent.glm_api_key_env = "TEST_ZHIPUAI_API_KEY"
    agent.glm_api_base = "https://example.test/v4"
    agent.glm_model = "glm-test"
    agent.glm_thinking = False
    agent.llm_stream = True
    agent.temperature = 0.3
    agent.ollama_base = "http://127.0.0.1:11434"
    agent.model = "ollama-test"
    agent.num_ctx = 1024
    agent._llm_cancel_tokens = CancellationTokenSlot()
    logger = _Logger()
    agent.get_logger = lambda: logger
    return agent, logger


class AgentCancellationBehaviourTests(unittest.TestCase):
    def test_user_interrupt_cancels_glm_without_ollama_fallback(self):
        agent, _ = _agent_for_request()

        def blocking_glm(**kwargs):
            self.assertFalse(kwargs["cancel_check"]())
            agent._on_tts_interrupt(agent_module.Bool(data=True))
            self.assertTrue(kwargs["cancel_check"]())
            raise VoiceBackendCancelled("cancelled")

        with mock.patch.dict(
            os.environ, {"TEST_ZHIPUAI_API_KEY": "test-only-value"}, clear=False
        ):
            with mock.patch.object(agent_module, "glm_chat", side_effect=blocking_glm):
                with mock.patch.object(agent_module, "ollama_chat") as ollama:
                    with self.assertRaises(VoiceBackendCancelled):
                        agent._request_llm(
                            [{"role": "user", "content": "测试"}],
                            max_tokens=32,
                        )

        ollama.assert_not_called()
        self.assertIsNone(agent._llm_cancel_tokens.current)

    def test_successful_request_releases_current_token(self):
        agent, _ = _agent_for_request()
        with mock.patch.dict(
            os.environ, {"TEST_ZHIPUAI_API_KEY": "test-only-value"}, clear=False
        ):
            with mock.patch.object(agent_module, "glm_chat", return_value="完成。"):
                result = agent._request_llm(
                    [{"role": "user", "content": "测试"}],
                    max_tokens=32,
                )

        self.assertEqual(result, "完成。")
        self.assertIsNone(agent._llm_cancel_tokens.current)


if __name__ == "__main__":
    unittest.main()
