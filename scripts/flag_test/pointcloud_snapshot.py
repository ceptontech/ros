"""Export exactly one PointCloud2, retaining field layout and point order."""

import json
import math
import struct


# sensor_msgs/msg/PointField types in ROS2 Jazzy.
FIELD_TYPES = {
    1: ("INT8", "b"),
    2: ("UINT8", "B"),
    3: ("INT16", "h"),
    4: ("UINT16", "H"),
    5: ("INT32", "i"),
    6: ("UINT32", "I"),
    7: ("FLOAT32", "f"),
    8: ("FLOAT64", "d"),
}


def write_snapshot(message, topic, output_path):
    """Decode advertised fields, including row padding and array components."""
    if message.width == 0 or message.height == 0:
        raise ValueError("Cannot plot an empty point cloud")
    if message.point_step <= 0:
        raise ValueError("point_step must be positive")
    if message.row_step < message.width * message.point_step:
        raise ValueError("row_step is smaller than a row of points")
    if len(message.data) < message.row_step * message.height:
        raise ValueError("PointCloud2 data is shorter than its advertised layout")

    fields = []
    byte_order = ">" if message.is_bigendian else "<"
    for field in message.fields:
        if field.datatype not in FIELD_TYPES:
            raise ValueError(f"Unsupported PointField datatype: {field.datatype}")
        type_name, format_code = FIELD_TYPES[field.datatype]
        decoder = struct.Struct(byte_order + format_code)
        if field.count < 1 or field.offset + decoder.size * field.count > message.point_step:
            raise ValueError(f"Field {field.name} does not fit in point_step")
        for component in range(field.count):
            values = []
            offset = field.offset + component * decoder.size
            for row in range(message.height):
                row_offset = row * message.row_step
                for column in range(message.width):
                    point_offset = row_offset + column * message.point_step
                    value = decoder.unpack_from(message.data, point_offset + offset)[0]
                    # JSON has no NaN/Inf. A null leaves a gap at the same index.
                    if isinstance(value, float) and not math.isfinite(value):
                        value = None
                    values.append(value)
            fields.append({
                "name": field.name if field.count == 1 else f"{field.name}[{component}]",
                "source_name": field.name,
                "component": component,
                "datatype": type_name,
                "offset": offset,
                "count": field.count,
                "values": values,
            })

    snapshot = {
        "topic": topic,
        "frame_id": message.header.frame_id,
        "stamp": f"{message.header.stamp.sec}.{message.header.stamp.nanosec:09d}",
        "width": message.width,
        "height": message.height,
        "point_count": message.width * message.height,
        "point_step": message.point_step,
        "row_step": message.row_step,
        "is_bigendian": message.is_bigendian,
        "is_dense": message.is_dense,
        "fields": fields,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as output:
        json.dump(snapshot, output, allow_nan=False, separators=(",", ":"))
