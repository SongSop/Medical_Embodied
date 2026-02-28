Example nodes live in `scripts/`.
- Publisher: `rosrun monitor monitor_mock_pub.py _battery_soc:=50 _fault_type:=localization _fault_severity:=1 _call_signal:=true`
- Anomaly service: `rosrun monitor anomaly_detect_server.py`
- Test scan: `rosservice call /detect_anomaly "mode: 0"`
- Test bed: `rosservice call /detect_anomaly "mode: 1\nbed_id: 0"`
