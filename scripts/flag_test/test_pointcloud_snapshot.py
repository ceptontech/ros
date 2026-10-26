"""Check decoding against deliberately padded and non-native point layouts."""

import json
from pathlib import Path
import struct
import tempfile
from types import SimpleNamespace
import unittest

from pointcloud_snapshot import FIELD_TYPES, write_snapshot


def make_message(**overrides):
    attributes = {
        "header": SimpleNamespace(frame_id="test", stamp=SimpleNamespace(sec=80, nanosec=123)),
        "width": 2, "height": 2, "point_step": 14, "row_step": 35,
        "is_bigendian": False, "is_dense": False,
        "fields": [
            SimpleNamespace(name="channel", offset=1, datatype=3, count=1),
            SimpleNamespace(name="vector", offset=5, datatype=7, count=2),
        ],
        "data": bytearray(70),
    }
    attributes.update(overrides)
    return SimpleNamespace(**attributes)


class SnapshotTests(unittest.TestCase):

    def export(self, message):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "data" / "frame.json"
            write_snapshot(message, "/points", output)
            return json.loads(output.read_text())

    def test_padding_arrays_nonfinite_and_endianness(self):
        for bigendian, byte_order in ((False, "<"), (True, ">")):
            with self.subTest(bigendian=bigendian):
                message = make_message(is_bigendian=bigendian)
                for index in range(4):
                    offset = (index // 2) * message.row_step + (index % 2) * message.point_step
                    struct.pack_into(byte_order + "h", message.data, offset + 1, index - 2)
                    struct.pack_into(byte_order + "ff", message.data, offset + 5,
                                     index + 0.5, [1.0, float("nan"), float("inf"), -2.0][index])
                result = self.export(message)
                self.assertEqual(result["stamp"], "80.000000123")
                self.assertEqual(result["point_count"], 4)
                self.assertEqual([field["name"] for field in result["fields"]],
                                 ["channel", "vector[0]", "vector[1]"])
                self.assertEqual(result["fields"][0]["values"], [-2, -1, 0, 1])
                self.assertEqual(result["fields"][1]["values"], [0.5, 1.5, 2.5, 3.5])
                self.assertEqual(result["fields"][2]["values"], [1.0, None, None, -2.0])

    def test_all_jazzy_field_types(self):
        values = [-128, 255, -32768, 65535, -2147483648, 4294967295, 1.25, -1.125]
        fields = []
        data = bytearray()
        for (datatype, (_, code)), value in zip(FIELD_TYPES.items(), values):
            fields.append(SimpleNamespace(name=f"type{datatype}", offset=len(data),
                                          datatype=datatype, count=1))
            data.extend(struct.pack("<" + code, value))
        message = make_message(width=1, height=1, point_step=len(data), row_step=len(data),
                               fields=fields, data=data)
        self.assertEqual([field["values"][0] for field in self.export(message)["fields"]], values)

    def test_rejects_invalid_layout_and_empty_cloud(self):
        for overrides in ({"data": bytearray(69)}, {"row_step": 27},
                          {"point_step": 12}, {"width": 0}):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                self.export(make_message(**overrides))


if __name__ == "__main__":
    unittest.main()
