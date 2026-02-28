#!/usr/bin/env python3
import rospy
from interfaces.srv import DetectAnomaly, DetectAnomalyResponse

DETECT_AREA = 0
DETECT_BED = 1


def _parse_int_csv(value):
    if not value:
        return []
    out = []
    for item in value.split(","):
        item = item.strip()
        if not item:
            continue
        try:
            out.append(int(item))
        except ValueError:
            out.append(0)
    return out


def handle(req):
    if req.mode == DETECT_AREA:
        beds = _parse_int_csv(rospy.get_param("~scan_beds", "0,2"))
        urgencies = _parse_int_csv(rospy.get_param("~scan_urgencies", "2,1"))
        while len(urgencies) < len(beds):
            urgencies.append(0)
        pairs = list(zip(beds, urgencies))
        pairs.sort(key=lambda x: x[1], reverse=True)
        bed_ids = [b for b, _ in pairs]
        urg = [u for _, u in pairs]
        details = "scan"
        return DetectAnomalyResponse(
            is_anomaly=bool(bed_ids),
            details=details,
            bed_ids=bed_ids,
            urgencies=urg,
        )

    bed_id = int(req.bed_id)
    force = rospy.get_param("~force_anomaly", "")
    if force.lower() in ("true", "1", "yes"):
        is_anomaly = True
    elif force.lower() in ("false", "0", "no"):
        is_anomaly = False
    else:
        is_anomaly = (bed_id % 2 == 0)

    details = "mock anomaly" if is_anomaly else "normal"
    rospy.loginfo("detect_anomaly mode=bed bed_id=%s -> %s", bed_id, details)
    return DetectAnomalyResponse(is_anomaly=is_anomaly, details=details, bed_ids=[], urgencies=[])


if __name__ == "__main__":
    rospy.init_node("anomaly_detect_server")
    rospy.Service("detect_anomaly", DetectAnomaly, handle)
    rospy.loginfo("anomaly_detect_server started")
    rospy.spin()
