# 2026-08-23 Voice Stabilization Lock Baseline

## Status

Rebecca Voice / Natural Navigation 已进入稳定基线阶段。

本版本停止继续扩大 Voice Stack 修改范围。
后续发现的问题原则上作为独立 Risk / Patch 处理，不回退当前已验证安全边界。

## Verified Capabilities

### Voice

- Normal conversation
- Wake / wake window
- L2 system sleep / wake
- L3 mute / unmute
- TTS interruption:
  - 停一下
  - 别说了
- TTS self-echo isolation

### Robot STOP

Robot STOP remains the highest-priority control path.

Verified:

- 停止移动
  -> Robot STOP

- 瑞贝卡，停止移动是什么意思？
  -> normal discussion
  -> no Robot STOP

- Rebecca TTS:
  停止移动就是……
  -> self-echo suppressed
  -> no Robot STOP

- User says 停止移动 while Rebecca is speaking
  -> Robot STOP
  -> /voice_input/input: 停止移动
  -> current TTS interrupted

Current policy:

1. exact Robot STOP always wins
2. restricted prefix STOP is allowed only for acoustic-boundary tolerance
3. prefix candidates matching Rebecca TTS are suppressed
4. ordinary discussion does not become Robot STOP

## Navigation

Verified:

- Natural navigation request
- nav_wait_confirm
- explicit user confirmation
- nav_confirmed
- command is not dispatched before confirmation
- confirmed navigation publishes:
  /voice_input/input -> 导航到<place>

Previous physical Voice -> Navigation -> Nav2 E2E baseline remains valid.

## Known Risks / Follow-up

### RISK-00 TTS-time merged navigation confirmation false positive

A real runtime case has been observed:

- Rebecca asks:
  `要让它过去吗？`
- TTS tail and following speech may be merged by ASR into:
  `过去吗？可以。`
- Current TTS-time suffix confirmation logic can canonicalize this as:
  `确认`
- This may dispatch navigation without a clean standalone user confirmation.

Required future behavior:

- TTS-time navigation confirmation should prefer exact confirmation only
- merged TTS-tail + confirmation utterances should be dropped rather than accepted
- user can repeat confirmation after TTS finishes
- safety takes priority over acoustic-boundary convenience

Priority: P0 before unrestricted physical Nav2 operation.

### RISK-01 Pending navigation timeout

Pending navigation confirmation can remain alive too long.

Required future behavior:

- add explicit timeout, recommended 10–15 s
- unrelated topic should clear pending navigation
- stale "是的" must never confirm an old navigation request

Priority: P0 before unrestricted physical deployment.

### RISK-02 L1 follow ASR robustness

Canonical:

- 瑞贝卡，暂停跟随
- 瑞贝卡，恢复跟随

Observed ASR may recognize 暂停跟随 as 停止跟随.

Need deterministic alias hardening without widening command matching dangerously.

### RISK-03 Natural navigation phrase coverage

Example:

- 让 Eric 回卧室
  -> navigation intent works

but:

- 你能让艾瑞克回卧室吗
  -> may fall through to LLM

Need expand deterministic navigation language coverage.

LLM must not claim that Eric is a human or that robot motion is impossible when a supported deterministic navigation intent exists.

### RISK-04 L2 wakeword ASR variation

Observed ASR variants include:

- 瑞菲卡
- 准贝卡

Canonical L2 commands can therefore occasionally miss deterministic matching.

Do not solve with unrestricted fuzzy matching.
Future solution should use a narrow wakeword canonicalization layer.

### RISK-05 nav_stop state persistence

After Robot STOP, /voice_agent/state can remain:

nav_stop

while subsequent normal conversation continues.

Verify whether nav_stop is intended as a persistent state or event-like state.

If event-like, return deterministically to chat_idle / appropriate idle state.

### RISK-06 Deterministic navigation safety guard

Navigation-shaped requests or malformed place aliases must not fall through to LLM and produce false claims such as successful movement / arrival.

Future patch should place deterministic navigation-shaped guarding before generic LLM chat.

### RISK-07 Test drift

Some historical unit tests may encode behavior that predates:

- TTS boundary latch
- short navigation confirmation handling
- current Robot STOP safety policy

Tests should be reconciled with the accepted safety specification.
Do not weaken production safety behavior merely to satisfy stale assertions.

### RISK-08 Localization robustness

Voice -> navigation dispatch is functionally established.

AMCL / localization robustness during physical navigation remains a separate mobile-base validation track and is not considered a Voice Stack defect.

## Test Gate Snapshot

Final regression run on 2026-08-23:

- 78 passed
- 7 failed
- 4 warnings

The baseline is intentionally locked without modifying production behavior
solely to make historical tests green.

### Failure Classification

1. `TtsConfirmGateTests.test_weak_confirms_do_not_pass`
   - Test/specification drift around TTS-time exact confirmation.
   - Current implementation accepts exact `是的` during `nav_wait_confirm`.
   - The historical test classifies `是的` as a weak confirmation.
   - Requires an explicit future safety-policy decision.

2. `AsrHardeningStaticTests.test_robot_stop_precedes_tts_control_gates`
   - Brittle static source assertion.
   - `_handle_robot_stop()` is still executed before TTS control gates,
     but its call is now multiline.

3. `UttRestrictedLatchPureTests.test_on_tts_speaking_true_resets_buffers`
   - Test stub drift.
   - Stub does not provide the newer `_apply_vad_mode_for_context()` helper.

4. `BoundaryIsolationTests.test_cross_boundary_merged_speech_never_published_normal`
   - Genuine known safety risk.
   - `去吗？可以。` can currently become canonical `确认`.
   - Covered by RISK-00.
   - Do not weaken this test merely to make the suite green.

5. `BoundaryIsolationTests.test_normal_min_utt_still_applies_without_latch`
   - Historical expectation conflicts with current `nav_control_fast`
     short-control-listening behavior introduced for `nav_wait_confirm`.

6. `LatchWiringStaticTests.test_endpoint_params_use_utt_latch`
   - Brittle static assertion predates the combined
     `_utt_restricted or nav_control_fast` condition.

7. `LatchWiringStaticTests.test_finalize_branch_uses_saved_latch_after_robot_stop`
   - Brittle source-string assertion predates the multiline
     `_handle_robot_stop()` call.

### Lock Decision

These failures are recorded as known risk / test debt.

Production code is not changed again in this baseline.

This baseline must not be treated as unrestricted physical Nav2 safe until
RISK-00 and RISK-01 are closed.

## Freeze Decision

The current Voice Stack is considered feature-complete for the current MVP.

No further opportunistic refactoring in this baseline.

Next work should proceed through isolated patches with:

1. PC-only regression
2. static/unit regression
3. explicit safety acceptance
4. physical Nav2 only after the above pass
