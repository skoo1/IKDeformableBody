# scan2smpl

본 폴더의 코드는 인체 스캔 데이터(`.obj`, `.ply`, `.stl`)로부터 **SMPL 모델의 형상 변수(shape, β), 자세(pose), 전역 위치(translation)** 를 추정하기 위한 코드이다.

SMPL 모델 표면과 스캔 데이터 표면 사이의 **유클리드 거리(Euclidean distance)** 및 **법선 벡터(normal) 거리**를 cost function으로 사용하며, 해당 cost를 최소화하도록 각 파라미터를 최적화한다.

---

## 최적화 변수

본 단계에서는 다음 파라미터를 최적화한다.

- shape (β)
- pose
- translation (trans)

---

## 실행 환경

- Python 3.8 (Anaconda 환경)
- SMPL 모델의 Python 구현이 포함된 `SMPL3/` 폴더
- 본 폴더 실행에 필요한 기타 Python 코드 및 의존성

---

## Cost Function (pyd)

본 폴더에서 사용되는 cost function은 **C++로 구현된 `.pyd` 파일**을 통해 계산된다.

- pyd 생성 위치: `pyd/scan2smpl/`
- pyd 파일은 CMA-ES 최적화를 빠르게 수행하기 위해 사용된다.

pyd 파일 생성 방법은 `pyd/scan2smpl/` 폴더의 README를 참조한다.

---

## 입력 및 출력

### 입력
- 인체 스캔 데이터 (`.obj`, `.ply`, `.stl`)
- SMPL 모델 파일 및 초기 파라미터

### 출력
- 추정된 SMPL 파라미터
  - shape (β)
  - pose
  - translation (trans)

입력 및 출력 파일 경로는 코드 내 설정을 통해 지정한다.


<여기 입력 파일, 출력 파일 경로 어떻게 할 지 코드 수정해서 추가 작성>