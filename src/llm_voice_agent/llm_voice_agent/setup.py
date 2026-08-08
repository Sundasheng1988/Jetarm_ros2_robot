from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'llm_voice_agent'

setup(
    name=package_name,
    version='0.0.3',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='sundasheng',
    maintainer_email='you@example.com',
    description='Voice agent with LLM + speech stack.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_voice_agent_node = llm_voice_agent.llm_voice_agent_node:main',
            'tts_speaker_node     = llm_voice_agent.tts_speaker_node:main',
            'executor_done_sayer  = llm_voice_agent.executor_done_sayer:main',
            'speech_dialog_funasr_node = llm_voice_agent.speech_dialog_funasr_node:main',
        ],
    },
)
