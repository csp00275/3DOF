//—— 설정 —————————————————————————
const int JOINTS = 5;                     // 관절 개수
float startA[JOINTS] = {90,   -180,   180,   0,   -90};  // 시작 각도
float endA  [JOINTS] = {90,  -90, 90, 90,  90};  // 목표 각도
float vMax = 45.0;       // 최대 속도 (deg/s)
float aMax = 90.0;       // 최대 가속도 (deg/s²)
unsigned long dt = 5;   // 샘플 간격 (ms)

float dA[JOINTS];        // 각 관절 이동량
float maxDist;           // 최대 이동량
//—— 내부 프로파일 파라미터 —————————————
float v_s, a_s;          // 정규화된 속도/가속도
float t_acc, t_const, T; // 가속시간, 크루즈시간, 총시간
float d_s_acc;           // 정규화된 가속 구간 거리

void setup() {
  Serial.begin(115200);
  delay(1000);  // Processing 쪽이 포트 열 시간 확보
  // 1. 이동량 계산
  maxDist = 0;
  for (int i = 0; i < JOINTS; i++) {
    dA[i] = fabs(endA[i] - startA[i]);
    if (dA[i] > maxDist) maxDist = dA[i];
  }
  // 2. 정규화 속도·가속도 설정 (s(t) in [0,1])
  v_s = vMax / maxDist;
  a_s = aMax / maxDist;
  // 3. 프로파일 타임 계산
  t_acc    = v_s / a_s;
  d_s_acc  = 0.5 * a_s * t_acc * t_acc;
  if (2*d_s_acc >= 1.0) {
    // 삼각 프로파일
    t_acc   = sqrt(1.0 / a_s);
    t_const = 0.0;
  } else {
    // 사다리꼴 프로파일
    t_const = (1.0 - 2*d_s_acc) / v_s;
  }
  T = 2*t_acc + t_const;
}

void loop() {
  // 4. 시간 순회하며 s(t) 계산
  for (float t = 0; t <= T; t += dt/1000.0) {
    float s;
    if (t < t_acc) {
      s = 0.5 * a_s * t * t;
    } else if (t < t_acc + t_const) {
      s = d_s_acc + v_s * (t - t_acc);
    } else {
      float td = T - t;
      s = 1.0 - 0.5 * a_s * td * td;
    }

    // 5차 다항 필터링 (주석 해제하면 가속도 연속성 UP)
    // float u = s;
    // s = 10*u*u*u - 15*u*u*u*u + 6*u*u*u*u*u;

    // 5개 관절에 적용
    for (int j = 0; j < JOINTS; j++) {
      float dir = (endA[j] >= startA[j]) ? 1.0 : -1.0;
      float theta = startA[j] + dir * dA[j] * s;
      Serial.print(theta, 2);
      if (j < JOINTS-1) Serial.print(',');
    }
    Serial.println();
    delay(dt);
  }

  // 방향 뒤집기 → 무한 왕복
  for (int i = 0; i < JOINTS; i++) {
    float tmp    = startA[i];
    startA[i]    = endA[i];
    endA[i]      = tmp;
    dA[i]        = fabs(endA[i] - startA[i]);
  }
  // maxDist, v_s, a_s, t_acc, t_const, T 재계산하거나
  // setup() 안 로직을 computeProfile()로 분리해 호출해도 됩니다.
}
