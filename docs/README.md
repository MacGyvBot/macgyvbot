# MacGyvBot Docs

이 폴더는 README에 넣기에는 긴 설계 문서, topic 계약, 파트별 계획, 실험
메모를 보관한다.

## 문서 구분

- `architecture/`: 패키지 경계, topic 계약, runtime 구조 같은 장기 유지 문서
- `vlm-advancement-plan.md`: VLM 개선 계획
- `vlm-latency-notes.md`: VLM grid latency 진단 메모
- 이슈/브랜치별 상세 기록은 필요할 때 별도 파일로 추가한다.

## 작성 기준

- README에는 설치, 빌드, 실행에 필요한 최소 정보만 둔다.
- 현재 runtime 구조 설명은 `EXPLAIN.md`에 둔다.
- 작업자/에이전트 공통 지침은 루트 `AGENTS.md`에 둔다.
- topic publisher/subscriber 또는 payload 계약이 바뀌면
  `architecture/topics.md`를 함께 수정한다.
