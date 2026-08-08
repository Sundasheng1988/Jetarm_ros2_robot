# JetArm Voice v1.1：GLM + CosyVoice 升级与无声验证指南

基线：`llm_voice_agent_current_260807.tar.gz`
升级包版本：v1.1（ROS package version `0.0.3`）

## 1. 本版改动

- 对话模型：`GLM 云端 → 本地 Ollama` 回退链。
- 默认 GLM：`glm-4.5-air`；可通过 launch 参数切换 `glm-4.7`。
- API Key 只读取环境变量 `ZHIPUAI_API_KEY`，不写入源码、launch、ROS 参数或日志。
- GLM 使用流式输出；完整句子可直接进入 TTS，减少首句等待。
- TTS：`CosyVoice HTTP → Piper` 回退链。
- Fun-CosyVoice3 输出按 `24000 Hz / mono / signed int16 PCM` 封装 WAV。
- 支持 `play_audio:=false`：只生成 WAV，不打开声卡或播放器。
- CosyVoice 合成期间收到打断，会停止后续写入并禁止播放残余音频。
- Piper `--sentence-silence` 修正为秒，默认 `0.14`。
- 任务文本只发布到 `/voice_input/input`，不再同时发送键盘入口。
- `task` 模式、任务补槽、确认和取消统一受唤醒窗口约束。
- `enable_robot_side_effects` 默认 `false`；默认不创建手势、face-follow 和舵机发布器。
- 新增 12 项标准库无声测试，无需启动 ROS、麦克风、播放器、GLM 或 CosyVoice。

## 2. 替换前备份

假设升级压缩包已下载到 `~/Downloads/llm_voice_agent_v1_1_260807.tar.gz`：

```bash
cd ~/ros2_ws/src

cp -a \
  llm_voice_agent \
  llm_voice_agent_backup_260807_before_glm_cosyvoice

tar -tzf ~/Downloads/llm_voice_agent_v1_1_260807.tar.gz
```

确认归档只有一个 `llm_voice_agent/` 根目录后再覆盖：

```bash
cd ~/ros2_ws/src

tar -xzf \
  ~/Downloads/llm_voice_agent_v1_1_260807.tar.gz
```

这个操作会覆盖同名源码并新增文件，但不会删除包内其他文件；完整备份仍保留在旁边。

## 3. 先运行无声单元测试

```bash
cd ~/ros2_ws/src/llm_voice_agent

python3 -m unittest discover \
  -v \
  -s test \
  -p 'test_voice_*.py'
```

期望结果：

```text
Ran 12 tests
OK
```

这些测试不会联网，也不会启动 ROS 或硬件。

## 4. 构建 ROS 包

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash

colcon build \
  --packages-select llm_voice_agent \
  --symlink-install

source install/setup.bash
```

确认新增 launch 已安装：

```bash
ls install/llm_voice_agent/share/llm_voice_agent/launch
```

应该能看到：

```text
glm_cosyvoice.launch.py
llm_voice_agent.launch.py
voice_stack.launch.py
```

## 5. 设置 GLM API Key

只在当前终端临时设置：

```bash
read -rsp 'Zhipu API key: ' ZHIPUAI_API_KEY
echo
export ZHIPUAI_API_KEY
```

确认“变量存在”而不显示密钥：

```bash
test -n "$ZHIPUAI_API_KEY" \
  && echo 'ZHIPUAI_API_KEY is set' \
  || echo 'ZHIPUAI_API_KEY is missing'
```

不要把密钥写入 launch、YAML、源码、Git 或终端截图。

## 6. 部署 CosyVoice 3 常驻服务

以下路径与本包默认 launch 一致：`/home/sundasheng/tools/CosyVoice`。

```bash
mkdir -p ~/tools
cd ~/tools

git clone --recursive \
  https://github.com/FunAudioLLM/CosyVoice.git

cd CosyVoice

conda create -n cosyvoice -y python=3.10
conda activate cosyvoice

pip install -r requirements.txt \
  -i https://mirrors.aliyun.com/pypi/simple/ \
  --trusted-host=mirrors.aliyun.com
```

下载推荐模型：

```bash
cd ~/tools/CosyVoice

python - <<'PY'
from modelscope import snapshot_download

snapshot_download(
    'FunAudioLLM/Fun-CosyVoice3-0.5B-2512',
    local_dir='pretrained_models/Fun-CosyVoice3-0.5B',
)
PY
```

启动常驻 FastAPI 服务：

```bash
conda activate cosyvoice
cd ~/tools/CosyVoice/runtime/python/fastapi

python server.py \
  --port 50000 \
  --model_dir ~/tools/CosyVoice/pretrained_models/Fun-CosyVoice3-0.5B
```

该终端应保持运行。默认零样本参考文件使用：

```text
/home/sundasheng/tools/CosyVoice/asset/zero_shot_prompt.wav
```

默认 `cosyvoice_prompt_text` 与官方参考音频配套：

```text
You are a helpful assistant.<|endofprompt|>希望你以后能够做的比我还好呦。
```

如果换成自己的参考音频，`cosyvoice_prompt_text` 必须改为该音频的准确转写文本，并保留 CosyVoice 3 所需的提示前缀与 `<|endofprompt|>`。

## 7. 当前环境可做的无声联调

`glm_cosyvoice.launch.py` 只启动 Agent 和 TTS，不启动 ASR、Parser、Grounding、Runtime 或硬件执行器；默认也不播放声音。

终端 A：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

export ZHIPUAI_API_KEY

ros2 launch llm_voice_agent glm_cosyvoice.launch.py \
  play_audio:=false \
  wav_output_dir:=/tmp/jetarm_tts_qa
```

终端 B：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once \
  /speech_query \
  std_msgs/msg/String \
  "{data: '你好，请用两句话介绍一下你自己'}"
```

检查生成文件：

```bash
ls -lht /tmp/jetarm_tts_qa
```

这一流程会消耗少量 GLM 额度并调用本机 CosyVoice，但不会打开播放器或连接任务执行链。

切换 GLM：

```bash
ros2 launch llm_voice_agent glm_cosyvoice.launch.py \
  glm_model:=glm-4.7 \
  play_audio:=false \
  wav_output_dir:=/tmp/jetarm_tts_qa
```

## 8. 完整语音栈

先保持直接机器人副作用关闭：

```bash
ros2 launch llm_voice_agent voice_stack.launch.py \
  glm_model:=glm-4.5-air \
  play_audio:=false \
  enable_robot_side_effects:=false
```

以后可以出声时再改为：

```bash
ros2 launch llm_voice_agent voice_stack.launch.py \
  glm_model:=glm-4.5-air \
  play_audio:=true \
  enable_robot_side_effects:=false
```

只有在手势和 face-follow 已单独验证后，才显式设置：

```text
enable_robot_side_effects:=true
```

注意：`enable_robot_side_effects` 只控制 Agent 直发的手势、face-follow 和舵机话题。任务文本仍可在用户确认后发布到 `/voice_input/input`；如果 Parser/Runtime/机械臂执行链同时运行，任务可能进入真实执行。因此无声开发时优先使用 `glm_cosyvoice.launch.py`。

## 9. 回退行为

- 未设置 `ZHIPUAI_API_KEY`、GLM 超时或返回错误：尝试本地 Ollama。
- Ollama 也不可用：播报“暂时无法连接语言模型，请稍后再试”。
- CosyVoice 服务、参考音频或请求失败：尝试 Piper。
- Piper 也失败：记录错误，不产生空 WAV。
- GLM 已经流式输出部分内容后断线：保留已输出内容，不再调用第二个模型重复播报。

如果只想使用 GLM、不回退 Ollama，可启动时传入：

```text
llm_fallback_backend:=none
```

## 10. 恢复旧版本

如果构建或联调不符合预期，先保留失败版本，再恢复备份：

```bash
cd ~/ros2_ws/src

mv \
  llm_voice_agent \
  llm_voice_agent_v1_1_failed_260807

mv \
  llm_voice_agent_backup_260807_before_glm_cosyvoice \
  llm_voice_agent

cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select llm_voice_agent \
  --symlink-install
```

## 11. 已验证与待实机验证

已验证：

- 全部 Python 文件语法编译。
- `package.xml` 可解析。
- launch 参数均已声明，无未解析 `LaunchConfiguration`。
- 12/12 无声测试通过。
- 压缩包不包含 API Key、`__pycache__` 或 `.pyc`。

当前交付环境没有 ROS2 Humble，因此仍需在你的 PC 上验证：

- `colcon build`。
- GLM 实际在线返回和额度扣减。
- CosyVoice 3 在 RTX 4060 上的模型加载、首包延迟和显存占用。
- 生成 WAV 的主观音色、韵律和读音。
- 最终 ASR/TTS 打断与唤醒窗口联调。

## 12. 官方参考

- GLM 思考模式与 `thinking.type`：https://docs.bigmodel.cn/cn/guide/capabilities/thinking-mode
- GLM-4.5 接口：https://docs.bigmodel.cn/cn/guide/models/text/glm-4.5
- CosyVoice 官方仓库：https://github.com/QwenAudio/CosyVoice
- Fun-CosyVoice3 模型：https://huggingface.co/FunAudioLLM/Fun-CosyVoice3-0.5B-2512
