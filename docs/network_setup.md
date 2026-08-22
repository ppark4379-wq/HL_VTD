# Network setup

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

To use another settings file without editing the default file:

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
