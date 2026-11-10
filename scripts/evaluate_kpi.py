"""KPI 4종 계산 (4주차).

    python scripts/evaluate_kpi.py --pred logs/robot_rx_*.csv --truth data/ground_truth/poses.csv --comm logs/comm_*.csv
    python scripts/evaluate_kpi.py ... --pick logs/sim_pick_*.csv     # Gazebo 확장을 했을 때만

    위치 MAE (mm)   : 예측 (x, y) 와 실측의 유클리드 거리 평균          기준 ≤ 2 mm
    각도 MAE (deg)  : vision_detector.angle_error_deg 평균 ([0,180) 정의)  기준 ≤ 3°
    송신 성공률      : ACK 받은 건수 / 송신 건수 (100회 연속)              기준 100 %
    검출→송신 P95   : comm 로그 latency_ms 의 95 백분위수                 기준 ≤ 300 ms
  (선택) 분류 정확도 : part_type 예측 = 실측 비율                          기준 ≥ 95 %
  (선택) Pick 성공률 : 사이클 중 물체가 맞는 상자에 들어간 비율 (Gazebo)   기준 ≥ 90 %

결과에는 측정 장비(CPU/GPU/카메라 해상도)를 같이 출력한다. 실측 전에는 달성으로 쓰지 않는다.
"""
import argparse


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--pred', required=True)
    p.add_argument('--truth', required=True)
    p.add_argument('--comm', required=True)
    p.add_argument('--pick', default='', help='ur5e_pick_place 의 /sim/pick_log CSV (선택)')
    args = p.parse_args()
    # TODO: 예측-실측을 sample_id 로 매칭 → 4개 지표 (+ 분류 정확도, --pick 이 있으면 Pick 성공률) 계산 → platform/cpu 정보와 함께 표로 출력
    raise NotImplementedError


if __name__ == '__main__':
    main()
