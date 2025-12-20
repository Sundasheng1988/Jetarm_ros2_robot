# setup.py  —— 直接整文件替换
from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'llm_voice_agent'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(),  # 确保有 src/llm_voice_agent/__init__.py
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装所有 launch 脚本（*.py），包含 voice_stack.launch.py / voice_stack_launch.py / llm_voice_agent.launch.py 等
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        # ✅ 新增：安装 config（把 YAML 安到 install 路径里）
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sundasheng',
    maintainer_email='you@example.com',
    description='Voice agent with LLM + speech stack.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 这些路径要和包内实际文件一致
            'llm_voice_agent_node = llm_voice_agent.llm_voice_agent_node:main',
            'tts_speaker_node     = llm_voice_agent.tts_speaker_node:main',
            'executor_done_sayer  = llm_voice_agent.executor_done_sayer:main',
            'speech_dialog_funasr_node = llm_voice_agent.speech_dialog_funasr_node:main',
        ],
    },
)

