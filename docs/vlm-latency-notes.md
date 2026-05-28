# VLM Latency Notes

이 문서는 `grasp_point_mode:=vlm` 실행 중 확인한 지연 원인과 로그 해석을
간단히 정리한다.

## 결론

`vlm` grid 방식은 한 번의 pick target refinement 안에서 VLM `generate()`를
여러 번 호출한다. 각 호출이 길게 걸리면 task queue가
`pick/refine_target_and_apply_vlm_yaw` step 안에서 오래 대기한다.

실시간 로봇 동작에는 `vlm_only` 또는 `vlm_only_smol`이 더 적합하다.
`vlm` grid 방식은 품질 비교나 오프라인 실험용에 가깝다.

## 확인된 병목

예시 로그에서 전처리 구간은 상대적으로 작았다.

- camera snapshot: 약 0.01초
- YOLO detect: 약 0.8초
- SAM VLM input build: 약 1.7초
- crop/PIL 변환: 거의 0초

주요 병목은 `VLM_TRACE model stage=ask_generate_start`부터
`ask_generate_done`까지의 VLM `model.generate()` 구간이었다.

예시:

```text
ask_id=2 generate: 12.075s
ask_id=3 generate: 57.939s
ask_id=4 generate: 103.380s
ask_id=5 generate: 304.455s
ask_id=6 generate: 204.168s
```

Python 코드 기준으로 병목은 아래 한 줄 안에 있다.

```python
generated_ids = self.model.generate(...)
```

이 호출 안에서 HuggingFace Transformers가 vision/text forward, token 생성 루프,
KV cache, attention/MLP, bitsandbytes CUDA kernel 등을 처리한다. 현재 로그 수준에서는
`generate()` 내부를 더 세분화해서 볼 수 없고, 바깥에서는 시작/종료 시각만 확인된다.

## `vlm` Grid 호출 구조

`grasp_point_mode:=vlm`은 내부적으로 여러 번 VLM을 호출한다.

일반적인 흐름:

1. `describe_grasp_context()`
   - 잡기 좋은 부위를 문장으로 설명하게 한다.
2. 3x3 grid cell 선택
   - 번호가 붙은 grid 중 grasp cell을 고르게 한다.
3. 4x4 grid cell 선택
   - 더 촘촘한 grid에서 다시 고르게 한다.
4. `_estimate_precise_point_and_yaw()`
   - `x_px`, `y_px`, `yaw_deg`를 JSON으로 받는다.
   - 파싱 실패 시 retry한다.
5. `_estimate_grasp_orientation_rpy()`
   - roll/pitch/yaw를 다시 추정한다.

`ask_id=1`은 launch/preload warmup일 수 있으므로, pick 중 로그는 보통
`ask_id=2`부터 보는 것이 편하다.

## Retry로 인한 긴 대기

예시 로그에서 `ask_id=5`, `ask_id=6`, `ask_id=7`은 orientation 단계가 아니라
`_estimate_precise_point_and_yaw()`의 retry였다.

판단 근거:

- 동일한 `prompt_chars=1259`
- 동일한 `input_tokens=1146`
- `region_precise_done` 로그가 아직 나오지 않음

즉 precise point/yaw JSON 파싱에 실패하면서 같은 prompt를 반복 호출한 것이다.
현재 이 retry는 최대 `max_retries_per_grid` 횟수까지 갈 수 있다.

## `parsed=False` 의미

`parsed=False`는 VLM 응답이 strict JSON으로 파싱되지 않았다는 뜻이다.
grid 단계에서는 텍스트에서 row/col/cell을 보조 추출해 `valid=True`가 될 수
있지만, precise point/yaw 단계에서는 `x_px`, `y_px`, `yaw_deg`가 모두 필요해
retry가 발생할 수 있다.

예시:

```text
VLM_TRACE model stage=ask_done ask_id=5 ... parsed=False
VLM_TRACE model stage=ask_start ask_id=6 ...
```

## 권장 실행 모드

실시간 pick 동작:

```bash
grasp_point_mode:=vlm_only_smol
```

VLM 없는 baseline 확인:

```bash
grasp_point_mode:=center
```

`vlm` grid 방식은 latency 실험이나 grasp 품질 비교가 필요할 때만 사용한다.

## `vlm_only_smol` 관찰

`vlm_only_smol`은 grid 방식과 달리 pick 중 VLM generate를 보통 한 번만 호출한다.
따라서 multi-call 문제는 줄어든다.

다만 관찰 로그에서는 단일 `generate()`도 약 81초 걸렸고, 응답은 strict JSON이
아니라 prompt 키워드 반복에 가까웠다.

예시:

```text
VLM_TRACE model stage=ask_generate_done ask_id=2 elapsed_sec=80.936
VLM_TRACE model stage=ask_done ask_id=2 ... parsed=False
VLM-only response did not contain valid x_px, y_px, yaw_deg
```

이 경우 VLM 결과는 실패하고 bbox center fallback으로 계속 진행한다.

## 추가 계측 방법

`generate()` 내부에서 무엇이 느린지 더 보고 싶다면 별도 계측이 필요하다.

- `max_new_tokens=1` 실험
  - 첫 token latency를 확인한다.
  - 이것도 길면 prefill/첫 forward가 병목일 가능성이 크다.
- Transformers streamer 사용
  - token이 나올 때마다 timestamp를 찍어 token별 생성 속도를 본다.
  - 첫 token 전 대기와 이후 token 반복 지연을 구분할 수 있다.
- PyTorch/CUDA profiler
  - kernel 단위 병목을 확인할 수 있다.
  - ROS 실행 중에는 비용과 복잡도가 크다.
- `model.forward()` 직접 호출
  - prefill과 decode step을 직접 나눠 볼 수 있다.
  - HuggingFace `generate()` 흐름을 일부 재구성해야 하므로 작업량이 크다.

## 개선 후보

`vlm` grid 방식을 계속 사용할 경우 검토할 후보:

- context 설명 호출 제거
- 4x4 grid 호출 제거
- precise retry 횟수 축소
- orientation RPY 호출 제거
- `max_new_tokens` 축소
- JSON-only prompt 강화
- `vlm_only`를 기본 실시간 경로로 두고 grid VLM은 실험 모드로 분리
