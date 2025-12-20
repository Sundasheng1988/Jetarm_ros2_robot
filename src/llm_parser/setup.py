from setuptools import setup, find_packages

package_name = 'llm_parser'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),  # 目录里有 llm_parser/...
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='sundasheng',
    maintainer_email='sundasheng@todo.todo',
    description='LLM command parser',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # 和 launch 里的 parser_exe 完全一致
            'llm_command_parser_node = llm_parser.llm_command_parser_node:main',
         
        ],
    },
)

