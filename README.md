# HL_VTD
Autonomous Driving Planning &amp; Control for HL VTD Simulation

## Overview

본 프로젝트는 VTD에서 제공되는 Ego Vehicle, Object,
Traffic Light 등의 GT 데이터를 기반으로
Planning 및 Vehicle Control 알고리즘을 개발하는 것을 목표로 합니다.

주요 개발 범위:

- VTD TCP/UDP 통신
- GT 및 OpenDRIVE 데이터 처리
- 교통법규 및 위험 상황 판단
- 경로 및 속도 Planning
- 횡/종방향 Vehicle Control
- 평가항목 기반 VTD Scenario 검증

---

## Architecture

VTD
 ↓
Network
 ↓
Data
 ↓
Planning
 ↓
Control
 ↓
VTD

Logger는 각 단계의 상태와 제어값을 기록합니다.

---

## Directory Structure

repo/
├── docs/        # 인터페이스 / 평가항목 / 연동 문서
├── config/      # 네트워크 / 차량 / 제어 파라미터
├── src/
│   ├── network/ # VTD 통신
│   ├── data/    # GT 및 도로 데이터 관리
│   ├── planning/# 상황 판단 및 경로/속도 계획
│   ├── control/ # 횡/종방향 제어
│   └── logger/  # 주행 로그
├── scenario/    # 자체 VTD 검증 시나리오
├── tools/       # 로그 분석 및 평가 도구
└── scripts/     # 빌드 / 실행

---

## Development

### Build

./scripts/build.sh

### Run

./scripts/run.sh

---

## Git Rule

main 브랜치는 항상 실행 가능한 코드를 유지합니다.

기능 개발 시 별도 브랜치를 생성합니다.

예:

feat/vtd-network
feat/planning-pedestrian
feat/lateral-control
fix/traffic-light
