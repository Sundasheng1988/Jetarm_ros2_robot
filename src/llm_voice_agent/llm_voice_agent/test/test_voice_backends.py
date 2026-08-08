#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import tempfile
import unittest
import wave
from unittest import mock

from llm_voice_agent import voice_backends


class FakeResponse:
    def __init__(self, *, status_code=200, json_data=None, lines=None, chunks=None):
        self.status_code = status_code
        self._json_data = json_data or {}
        self._lines = lines or []
        self._chunks = chunks or []
        self.headers = {"x-request-id": "test-request"}
        self.closed = False

    def json(self):
        return self._json_data

    def iter_lines(self, decode_unicode=False):
        yield from self._lines

    def iter_content(self, chunk_size=32768):
        yield from self._chunks

    def close(self):
        self.closed = True


class FakeRequests:
    def __init__(self, response):
        self.response = response
        self.calls = []

    def post(self, url, **kwargs):
        self.calls.append((url, kwargs))
        return self.response


class PartialStreamResponse(FakeResponse):
    def iter_lines(self, decode_unicode=False):
        yield 'data: {"choices":[{"delta":{"content":"已经收到。"}}]}'.encode('utf-8')
        raise ConnectionError("stream interrupted")


class SentenceAccumulatorTests(unittest.TestCase):
    def test_emits_complete_sentences_then_flushes_remainder(self):
        emitted = []
        accumulator = voice_backends.SentenceAccumulator(emitted.append)
        accumulator.feed("你好，")
        accumulator.feed("我是瑞贝卡。今天")
        self.assertEqual(emitted, ["你好，我是瑞贝卡。"])
        accumulator.flush()
        self.assertEqual(emitted, ["你好，我是瑞贝卡。", "今天"])


class GlmTests(unittest.TestCase):
    def test_non_streaming_glm_request(self):
        response = FakeResponse(
            json_data={"choices": [{"message": {"content": "你好。"}}]}
        )
        fake_http = FakeRequests(response)
        with mock.patch.object(voice_backends, "requests", fake_http):
            result = voice_backends.glm_chat(
                api_base="https://example.test/v4",
                api_key="secret",
                model="glm-test",
                messages=[{"role": "user", "content": "你好"}],
                temperature=0.4,
                max_tokens=64,
                timeout=(1.0, 2.0),
                thinking=False,
                stream=False,
            )
        self.assertEqual(result, "你好。")
        _, kwargs = fake_http.calls[0]
        self.assertEqual(kwargs["headers"]["Authorization"], "Bearer secret")
        self.assertEqual(kwargs["json"]["thinking"], {"type": "disabled"})
        self.assertTrue(response.closed)

    def test_streaming_glm_emits_sentences(self):
        events = [
            {"choices": [{"delta": {"content": "第一句。"}}]},
            {"choices": [{"delta": {"content": "第二句"}}]},
        ]
        lines = [
            f"data: {json.dumps(event, ensure_ascii=False)}".encode()
            for event in events
        ]
        lines.append(b"data: [DONE]")
        fake_http = FakeRequests(FakeResponse(lines=lines))
        emitted = []
        with mock.patch.object(voice_backends, "requests", fake_http):
            result = voice_backends.glm_chat(
                api_base="https://example.test/v4",
                api_key="secret",
                model="glm-test",
                messages=[{"role": "user", "content": "测试"}],
                temperature=0.4,
                max_tokens=64,
                timeout=(1.0, 2.0),
                thinking=False,
                stream=True,
                on_sentence=emitted.append,
            )
        self.assertEqual(result, "第一句。第二句")
        self.assertEqual(emitted, ["第一句。", "第二句"])

    def test_partial_stream_is_kept_instead_of_repeated_by_fallback(self):
        fake_http = FakeRequests(PartialStreamResponse())
        emitted = []
        with mock.patch.object(voice_backends, "requests", fake_http):
            result = voice_backends.glm_chat(
                api_base="https://example.test/v4",
                api_key="secret",
                model="glm-test",
                messages=[{"role": "user", "content": "测试"}],
                temperature=0.4,
                max_tokens=64,
                timeout=(1.0, 2.0),
                thinking=False,
                stream=True,
                on_sentence=emitted.append,
            )
        self.assertEqual(result, "已经收到。")
        self.assertEqual(emitted, ["已经收到。"])

    def test_http_error_does_not_expose_response_body(self):
        response = FakeResponse(status_code=401)
        fake_http = FakeRequests(response)
        with mock.patch.object(voice_backends, "requests", fake_http):
            with self.assertRaises(voice_backends.VoiceBackendError) as caught:
                voice_backends.glm_chat(
                    api_base="https://example.test/v4",
                    api_key="secret-key",
                    model="glm-test",
                    messages=[{"role": "user", "content": "测试"}],
                    temperature=0.4,
                    max_tokens=64,
                    timeout=(1.0, 2.0),
                    stream=False,
                )
        self.assertIn("HTTP 401", str(caught.exception))
        self.assertNotIn("secret-key", str(caught.exception))


class CosyVoiceTests(unittest.TestCase):
    def test_raw_pcm_is_wrapped_as_24khz_wav_across_odd_chunks(self):
        pcm = b"\x00\x00\x01\x00" * 100
        response = FakeResponse(chunks=[pcm[:121], pcm[121:]])
        fake_http = FakeRequests(response)
        with tempfile.TemporaryDirectory() as tmpdir:
            wav_path = os.path.join(tmpdir, "out.wav")
            with mock.patch.object(voice_backends, "requests", fake_http):
                count = voice_backends.cosyvoice_to_wav(
                    base_url="http://127.0.0.1:50000",
                    mode="sft",
                    text="你好",
                    wav_path=wav_path,
                    sample_rate=24000,
                    speaker_id="speaker-test",
                )
            self.assertEqual(count, len(pcm))
            with wave.open(wav_path, "rb") as audio:
                self.assertEqual(audio.getnchannels(), 1)
                self.assertEqual(audio.getsampwidth(), 2)
                self.assertEqual(audio.getframerate(), 24000)
                self.assertEqual(audio.getnframes(), len(pcm) // 2)

    def test_zero_shot_requires_prompt(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            with self.assertRaises(voice_backends.VoiceBackendError):
                voice_backends.cosyvoice_to_wav(
                    base_url="http://127.0.0.1:50000",
                    mode="zero_shot",
                    text="你好",
                    wav_path=os.path.join(tmpdir, "out.wav"),
                    sample_rate=24000,
                )

    def test_interrupt_cancels_stream_and_removes_partial_wav(self):
        pcm = b"\x00\x00" * 100
        response = FakeResponse(chunks=[pcm[:100], pcm[100:]])
        fake_http = FakeRequests(response)
        checks = {"count": 0}

        def cancel_check():
            checks["count"] += 1
            return checks["count"] >= 3

        with tempfile.TemporaryDirectory() as tmpdir:
            wav_path = os.path.join(tmpdir, "out.wav")
            with mock.patch.object(voice_backends, "requests", fake_http):
                with self.assertRaises(voice_backends.VoiceBackendCancelled):
                    voice_backends.cosyvoice_to_wav(
                        base_url="http://127.0.0.1:50000",
                        mode="sft",
                        text="你好",
                        wav_path=wav_path,
                        sample_rate=24000,
                        speaker_id="speaker-test",
                        cancel_check=cancel_check,
                    )
            self.assertFalse(os.path.exists(wav_path))
            self.assertTrue(response.closed)


if __name__ == "__main__":
    unittest.main()
