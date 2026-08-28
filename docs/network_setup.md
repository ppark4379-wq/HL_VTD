# Network setup

## 2026 대회 공지 설정

2026-08-27 대회 안내문에 공지된 네트워크는 다음과 같습니다.

```yaml
vtd:
  host_ip: "192.168.50.11"
  tcp_port: 9910
  rtsp_port: 8554

controller:
  ip: "192.168.50.10"
  udp_port: 9912
```

두 PC의 서브넷 마스크는 `255.255.255.0`입니다. 현재 저장소의 기본
`config/network.yaml`은 연구실 연동 주소이므로 대회 환경과 혼동하지
않아야 합니다.

## 설정 파일

The program loads all network addresses from `config/network.yaml` at startup.

```yaml
vtd:
  host_ip: "165.246.170.76"
  tcp_port: 9910
  rtsp_port: 8554

controller:
  ip: "165.246.170.86"
  udp_port: 9912
```

- `vtd.host_ip`: VTD host address used by TCP and RTSP clients
- `vtd.tcp_port`: VTD TCP server port
- `vtd.rtsp_port`: VTD video stream port
- `controller.ip`: controller PC address to which the host sends UDP data
- `controller.udp_port`: controller PC UDP receive port

Build and run from the repository root:

```bash
bash scripts/build.sh
bash scripts/run.sh
```

HL_VTD 바이너리를 직접 실행할 때 다른 설정 파일을 사용하려면:

```bash
./build/hl_vtd --config /path/to/network.yaml
```

Validate the file without opening a TCP connection:

```bash
./build/hl_vtd --check-config
```

All five settings are required. Ports must be integers from 1 through 65535.
The program validates the file before attempting a TCP connection and reports a
configuration error with its line number when possible.
